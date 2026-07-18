import time
import framebuf
import sensor
import image
import math
import ujson
from machine import SoftI2C, Pin
from pyb import Timer, UART
from micropython import const

# ====================== 1. 硬件引脚定义 ======================
I2C_SDA_PIN = 'P5'
I2C_SCL_PIN = 'P4'
LED_STATUS = 'P3'   # 正常常亮，报警时熄灭
LED_ALARM = 'P2'    # 正常熄灭，报警时亮起
BUZZER_PIN = 'P7'   # 正常静音，报警时鸣响
ESP8266_TX_PIN = 'P0'
ESP8266_RX_PIN = 'P1'

# ====================== 2. 核心检测配置 ======================
FIRE_LAB_THRESHOLD =(29, 100, -29, 46, 15, 60)
FIRE_PIXEL_THRESHOLD = 10
FIRE_AREA_THRESHOLD = 15
FIRE_MERGE = True
FIRE_DETECT_INTERVAL = 80

MIN_ASPECT_RATIO = 1.0
MAX_ASPECT_RATIO = 2.5
MAX_CIRCULARITY = 0.65
MIN_CIRCULARITY = 0.3
MIN_EDGE_RATIO = 0.4
MAX_CORE_DENSITY = 0.4
TEMP_ALARM_THRESHOLD = 50.0

# ====================== 3. ESP8266 AP模式配置 =======================
AP_SSID = "ESP8266_AP"
AP_PASSWORD = "YOUR_AP_PASSWORD"
AP_CHANNEL = 6
AP_ENCRYPT = 3
TCP_SERVER_PORT = 8080
ESP8266_BAUDRATE = 115200
WIFI_SEND_INTERVAL = 1000       # WiFi发送最小间隔（毫秒）
HEARTBEAT_INTERVAL = 5000       # D. 心跳包发送间隔（毫秒）
HEARTBEAT_MAX_MISS = 3          # D. 连续心跳失败次数上限
HEARTBEAT_TIMEOUT = 300         # C. 心跳包等待超时（毫秒）
UART_TIMEOUT = 200              # B. UART 单次等待超时（毫秒）
ESP8266_RESET_THRESHOLD = 5     # E. ESP8266 重启阈值

# ====================== 3b. 火焰投票 & 曝光 & 日志配置 =======================
FIRE_VOTE_WINDOW = 5            # C. 滑动投票窗口帧数
FIRE_VOTE_THRESHOLD = 3         # C. 触发报警所需最少票数
EXPOSURE_MIN = 2000             # A. 最低曝光时间（微秒）
EXPOSURE_MAX = 20000            # A. 最高曝光时间（微秒）
EXPOSURE_INIT = 5000            # A. 初始曝光时间（微秒）
EXPOSURE_ADJUST_INTERVAL = 3000 # A. 曝光调整周期（毫秒）
EXPOSURE_STEP = 1500            # A. 每次调整步长（微秒）
LOG_FILE = "/sd/fire_alarm.log" # E. SD卡日志路径

# ====================== 4. 装饰器 & 工具函数 ======================
def i2c_retry(max_retry=5, delay_ms=20):
    def decorator(func):
        def wrapper(*args, **kwargs):
            retry = max_retry
            while retry > 0:
                try:
                    return func(*args, **kwargs)
                except OSError as e:
                    if e.errno == 19:
                        retry -= 1
                        time.sleep_ms(delay_ms)
                    else:
                        raise
            print(f"I2C操作失败：{func.__name__} 重试{max_retry}次仍失败")
            return None
        return wrapper
    return decorator

# ====================== 5. ESP8266 AT指令函数 =======================
uart_esp8266 = UART(1, ESP8266_BAUDRATE, timeout_char=1000)
client_connected = False
last_wifi_send_time = 0       # 上次WiFi发送时间戳
last_heartbeat_time = 0       # D. 上次心跳时间戳
heartbeat_miss_count = 0      # D. 连续心跳失败计数
last_exposure_adjust_time = 0 # A. 上次曝光调整时间戳
current_exposure_us = EXPOSURE_INIT  # A. 当前曝光值（微秒）
last_esp8266_reset_time = 0   # E. 上次 ESP8266 重启时间
esp8266_hard_fail_count = 0   # E. ESP8266 硬故障计数

def uart_clear_buffer():
    """B. 清空 UART 缓冲区"""
    try:
        while uart_esp8266.any():
            uart_esp8266.read()
    except:
        pass
    time.sleep_ms(10)

def uart_wait_response(expected_resp="OK", timeout=2000, debug=False):
    """B. 改进的 UART 响应等待函数"""
    resp = ""
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < timeout:
        try:
            if uart_esp8266.any():
                data = uart_esp8266.read()
                if data:
                    try:
                        resp += data.decode('utf-8', 'ignore')
                    except:
                        pass
                if expected_resp in resp:
                    if debug:
                        print(f"✅ 收到期望响应：{expected_resp}")
                    return True, resp
        except:
            pass
        time.sleep_ms(5)

    if debug:
        print(f"⚠️  未收到期望响应 '{expected_resp}'，超时 {timeout}ms")
        print(f"   实际响应：{resp[:100]}")
    return False, resp

def send_at_cmd(cmd, timeout=2000, expected_resp="OK", debug=False):
    """B. 改进的 AT 指令发送"""
    uart_clear_buffer()

    if debug:
        print(f"📤 发送 AT 指令：{cmd}")

    try:
        uart_esp8266.write(f"{cmd}\r\n")
        time.sleep_ms(50)
        success, resp = uart_wait_response(expected_resp, timeout, debug)
        return success, resp
    except Exception as e:
        print(f"❌ AT 指令异常：{e}")
        return False, str(e)

def config_esp8266_as_ap(debug=False):
    """配置 ESP8266 为 AP 模式"""
    print("===== 开始配置 ESP8266 为 AP 模式 =====")

    success, resp = send_at_cmd("AT", timeout=2000, expected_resp="OK", debug=debug)
    if not success:
        print(f"❌ 模块通信失败")
        return False
    print("✅ 1. AT 指令测试成功")

    success, resp = send_at_cmd("AT+CWMODE=2", timeout=2000, expected_resp="OK", debug=debug)
    if not success:
        print(f"❌ 设置 AP 模式失败")
        return False
    print("✅ 2. AP 模式设置成功")

    ap_cmd = f'AT+CWSAP="{AP_SSID}","{AP_PASSWORD}",{AP_CHANNEL},{AP_ENCRYPT}'
    success, resp = send_at_cmd(ap_cmd, timeout=3000, expected_resp="OK", debug=debug)
    if not success:
        print(f"❌ 配置 AP 热点失败")
        return False
    print(f"✅ 3. AP 热点配置完成：SSID={AP_SSID}，密码={AP_PASSWORD}")

    send_at_cmd("AT+CIPMUX=1", timeout=2000, expected_resp="OK")
    send_at_cmd("AT+CIPSERVER=0", timeout=2000, expected_resp="OK")

    server_cmd = f"AT+CIPSERVER=1,{TCP_SERVER_PORT}"
    success, resp = send_at_cmd(server_cmd, timeout=5000, expected_resp="OK", debug=debug)
    if not success:
        print(f"❌ 开启 TCP 服务器失败")
        return False
    print(f"✅ 4. TCP 服务器已开启：IP=192.168.4.1，端口={TCP_SERVER_PORT}")

    return True

def check_esp8266_client():
    """F. 改进的客户端连接检测"""
    global client_connected
    try:
        # 尝试多次读取缓冲区，确保不遗漏连接信息
        for _ in range(3):
            if uart_esp8266.any():
                data = uart_esp8266.read()
                if data:
                    try:
                        resp = data.decode('utf-8', 'ignore')
                        if "CONNECT" in resp:
                            if not client_connected:
                                client_connected = True
                                print("✅ TCP 客户端已连接！")
                        elif "CLOSED" in resp:
                            if client_connected:
                                client_connected = False
                                print("❌ TCP 客户端已断开！")
                    except:
                        pass
            time.sleep_ms(5)
    except Exception as e:
        print(f"⚠️  客户端检测异常：{e}")

def send_sensor_data_to_wifi(temp, hum, fire_detected, alarm_triggered):
    """D. 改进的 WiFi 数据发送"""
    global client_connected, last_wifi_send_time

    if not client_connected:
        return False

    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_wifi_send_time) < WIFI_SEND_INTERVAL:
        return False

    last_wifi_send_time = current_time

    fire_flag = 1 if fire_detected else 0
    alarm_flag = 1 if alarm_triggered else 0
    data_str = ujson.dumps({"temp": temp, "hum": hum, "fire": fire_flag, "alarm": alarm_flag})

    try:
        uart_clear_buffer()

        # 发送 CIPSEND 指令
        uart_esp8266.write(f"AT+CIPSEND=0,{len(data_str)}\r\n")
        time.sleep_ms(50)

        # 等待 '>' 提示符
        got_prompt = False
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < UART_TIMEOUT:
            if uart_esp8266.any():
                raw = uart_esp8266.read()
                if raw:
                    try:
                        resp = raw.decode('utf-8', 'ignore')
                        if ">" in resp:
                            got_prompt = True
                            break
                    except:
                        pass
            time.sleep_ms(5)

        if not got_prompt:
            print("⚠️  WiFi：未收到 CIPSEND 提示符")
            client_connected = False
            return False

        # 发送数据
        uart_esp8266.write(data_str)
        print(f"📤 WiFi 数据已发送：{data_str}")
        return True

    except Exception as e:
        print(f"❌ WiFi 数据发送异常：{e}")
        client_connected = False
        return False

# ====================== 5b. 心跳包 & TCP 重连 ======================
def send_heartbeat():
    """C. 改进的心跳包发送"""
    global client_connected, last_heartbeat_time, heartbeat_miss_count, esp8266_hard_fail_count

    if not client_connected:
        return

    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_heartbeat_time) < HEARTBEAT_INTERVAL:
        return

    last_heartbeat_time = current_time
    ping_str = '{"type":"ping"}'

    try:
        uart_clear_buffer()

        # 发送 CIPSEND 指令
        uart_esp8266.write(f"AT+CIPSEND=0,{len(ping_str)}\r\n")
        time.sleep_ms(30)

        # 等待 '>' 提示符（使用更长的超时）
        got_prompt = False
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < HEARTBEAT_TIMEOUT:
            if uart_esp8266.any():
                raw = uart_esp8266.read()
                if raw:
                    try:
                        resp = raw.decode('utf-8', 'ignore')
                        if ">" in resp:
                            got_prompt = True
                            break
                    except:
                        pass
            time.sleep_ms(5)

        if got_prompt:
            # 发送心跳数据
            uart_esp8266.write(ping_str)
            heartbeat_miss_count = 0
            esp8266_hard_fail_count = 0
            print("💓 心跳已发送")
            return True
        else:
            heartbeat_miss_count += 1
            print(f"⚠️  心跳失败 ({heartbeat_miss_count}/{HEARTBEAT_MAX_MISS})")

            if heartbeat_miss_count >= HEARTBEAT_MAX_MISS:
                print("❌ 连续心跳超时，尝试重启 TCP 服务器...")
                client_connected = False
                heartbeat_miss_count = 0
                restart_tcp_server()
            return False

    except Exception as e:
        print(f"❌ 心跳包异常：{e}")
        heartbeat_miss_count += 1
        return False

def restart_tcp_server():
    """重启 TCP 服务器"""
    global client_connected
    print("🔄 重启 TCP 服务器...")
    try:
        send_at_cmd("AT+CIPSERVER=0", timeout=2000, expected_resp="OK")
        time.sleep_ms(500)
        success, _ = send_at_cmd(f"AT+CIPSERVER=1,{TCP_SERVER_PORT}", timeout=5000, expected_resp="OK")
        if success:
            print("✅ TCP 服务器已重启，等待新连接")
        else:
            print("❌ TCP 服务器重启失败")
        client_connected = False
    except Exception as e:
        print(f"❌ TCP 重启异常：{e}")

# ====================== 5c. E. ESP8266 健���监控 ======================
def monitor_esp8266_health():
    """E. 监控 ESP8266 健康状态，自动修复"""
    global client_connected, heartbeat_miss_count, esp8266_hard_fail_count, last_esp8266_reset_time

    current_time = time.ticks_ms()

    # 如果连续心跳失败过多，执行硬重启
    if heartbeat_miss_count > HEARTBEAT_MAX_MISS + 2:
        esp8266_hard_fail_count += 1

        # 防止频繁重启（至少间隔 30 秒）
        if time.ticks_diff(current_time, last_esp8266_reset_time) > 30000:
            print(f"🚨 WiFi 连接故障（{esp8266_hard_fail_count}次），执行硬重启...")
            try:
                uart_clear_buffer()
                uart_esp8266.write("AT+RST\r\n")
                print("⏳ ESP8266 重启中，等待 3 秒...")
                time.sleep_ms(3000)

                # 重新配置
                if config_esp8266_as_ap(debug=False):
                    heartbeat_miss_count = 0
                    client_connected = False
                    last_esp8266_reset_time = current_time
                    print("✅ ESP8266 已恢复")
                else:
                    print("❌ ESP8266 恢复失败")
            except Exception as e:
                print(f"❌ WiFi 硬重启异常：{e}")

# ====================== 6. CRC8校验函数 ======================
def AHT20_crc8_check(data):
    crc8 = 0xFF
    polynom = 0x31
    for i in range(6):
        crc8 ^= data[i]
        for _ in range(8):
            if crc8 & 0x80:
                crc8 = (crc8 << 1) ^ polynom
            else:
                crc8 <<= 1
            crc8 &= 0xFF
    return crc8 == data[6]

# ====================== 7. AHT20驱动（容错版） ======================
class AHT20:
    AHT20_I2CADDR = 0x38
    AHT20_CMD_SOFTRESET = bytearray([0xBA])
    AHT20_CMD_INITIALIZE = bytearray([0xBE, 0x08, 0x00])
    AHT20_CMD_MEASURE = bytearray([0xAC, 0x33, 0x00])
    AHT20_STATUSBIT_BUSY = 7
    AHT20_STATUSBIT_CALIBRATED = 3

    def __init__(self, i2c):
        self.i2c = i2c
        self.temp = 0.0
        self.hum = 0.0
        self.last_read_time = 0
        self.read_interval = 2000
        try:
            self.soft_reset()
            if not self.get_calibrated_status():
                self.initialize()
                time.sleep_ms(50)
        except Exception as e:
            print(f"AHT20初始化警告：{e}")

    @i2c_retry(max_retry=5, delay_ms=20)
    def soft_reset(self):
        self.i2c.writeto(self.AHT20_I2CADDR, self.AHT20_CMD_SOFTRESET)
        time.sleep_ms(60)

    @i2c_retry(max_retry=5, delay_ms=20)
    def initialize(self):
        self.i2c.writeto(self.AHT20_I2CADDR, self.AHT20_CMD_INITIALIZE)

    @i2c_retry(max_retry=5, delay_ms=20)
    def get_status(self):
        status = self.i2c.readfrom(self.AHT20_I2CADDR, 1)
        return status[0] if status else None

    def get_calibrated_status(self):
        status = self.get_status()
        return status is not None and ((status >> self.AHT20_STATUSBIT_CALIBRATED) & 0x01)

    def get_busy_status(self):
        status = self.get_status()
        return status is not None and ((status >> self.AHT20_STATUSBIT_BUSY) & 0x01)

    @i2c_retry(max_retry=5, delay_ms=20)
    def read_measure_data(self):
        time.sleep_ms(10)
        self.i2c.writeto(self.AHT20_I2CADDR, self.AHT20_CMD_MEASURE)
        time.sleep_ms(100)
        timeout = 500
        while self.get_busy_status() == 1 and timeout > 0:
            time.sleep_ms(20)
            timeout -= 20
        return self.i2c.readfrom(self.AHT20_I2CADDR, 7)

    def update_data(self):
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_read_time) >= self.read_interval:
            try:
                data = self.read_measure_data()
                if data and len(data) == 7 and AHT20_crc8_check(data):
                    self.hum = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) * 100 / (2**20)
                    self.temp = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * 200 / (2**20) - 50
                    self.temp = round(self.temp, 1)
                    self.hum = round(self.hum, 1)
                self.last_read_time = current_time
            except Exception as e:
                print(f"AHT20读取警告：{e}（使用默认值）")
        return self.temp, self.hum

# ====================== 8. SSD1306 OLED驱动 ======================
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)

class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,
            SET_MEM_ADDR, 0x00,
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xff,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def fill(self, col):
        self.framebuf.fill(col)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.buffer = bytearray(((height // 8) * width) + 1)
        self.buffer[0] = 0x40
        self.framebuf = framebuf.FrameBuffer1(memoryview(self.buffer)[1:], width, height)
        super().__init__(width, height, external_vcc)

    @i2c_retry(max_retry=5, delay_ms=20)
    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    @i2c_retry(max_retry=5, delay_ms=20)
    def write_framebuf(self):
        self.i2c.writeto(self.addr, self.buffer)

    def poweron(self):
        pass

# ====================== 9. 火焰&温度报警模块 ======================
class FireAlarm:
    def __init__(self, led_status_pin, led_alarm_pin, buzzer_pin):
        self.led_status = Pin(led_status_pin, Pin.OUT, value=1)
        self.led_alarm = Pin(led_alarm_pin, Pin.OUT, value=0)
        self.tim = Timer(4, freq=2000)
        self.buzzer_pwm = self.tim.channel(1, Timer.PWM, pin=Pin(buzzer_pin))
        self.buzzer_pwm.pulse_width_percent(0)

        self.fire_threshold = FIRE_LAB_THRESHOLD
        self.last_detect_time = 0
        self.detect_interval = FIRE_DETECT_INTERVAL
        self._last_fire_detected = False
        self._vote_window = []

    def _calc_circularity(self, blob):
        pixels = blob.pixels()
        w = blob.w()
        h = blob.h()
        perimeter = 2 * (w + h)
        if perimeter == 0:
            return 0.0
        circularity = (4 * math.pi * pixels) / (perimeter * perimeter)
        return circularity

    def _is_fire_shape(self, blob):
        w = blob.w()
        h = blob.h()
        if w == 0:
            return False
        aspect_ratio = h / w
        circularity = self._calc_circularity(blob)
        if (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO) and (MIN_CIRCULARITY <= circularity < MAX_CIRCULARITY):
            return True
        return False

    def detect_fire(self, img, temp):
        current_time = time.ticks_ms()
        fire_detected = self._last_fire_detected

        if time.ticks_diff(current_time, self.last_detect_time) >= self.detect_interval:
            blobs = img.find_blobs(
                [self.fire_threshold],
                area_threshold=FIRE_AREA_THRESHOLD,
                pixels_threshold=FIRE_PIXEL_THRESHOLD,
                merge=FIRE_MERGE,
                margin=1,
                x_stride=1,
                y_stride=1
            )
            valid_fire_blobs = []
            for b in blobs:
                if b.density() > 0.4 and self._is_fire_shape(b):
                    valid_fire_blobs.append(b)
                    img.draw_rectangle(b.rect(), color=(255, 0, 0), thickness=2)
                    img.draw_cross(b.cx(), b.cy(), color=(0, 255, 0), size=4)
                    img.draw_string(b.x(), b.y()-10, f"AR:{b.h()/b.w():.1f}", color=(255,255,0))
                    img.draw_string(b.x(), b.y()-20, f"C:{self._calc_circularity(b):.1f}", color=(255,255,0))

            raw_fire = len(valid_fire_blobs) > 0
            self._vote_window.append(1 if raw_fire else 0)
            if len(self._vote_window) > FIRE_VOTE_WINDOW:
                self._vote_window.pop(0)
            fire_detected = sum(self._vote_window) >= FIRE_VOTE_THRESHOLD
            self._last_fire_detected = fire_detected
            self.last_detect_time = current_time

        alarm_triggered = fire_detected or (temp > TEMP_ALARM_THRESHOLD)

        if alarm_triggered:
            self.led_status.value(0)
            self.led_alarm.value(1)
            self.buzzer_pwm.pulse_width_percent(80)
        else:
            self.led_status.value(1)
            self.led_alarm.value(0)
            self.buzzer_pwm.pulse_width_percent(0)

        return fire_detected, alarm_triggered

# ====================== 9b. 辅助功能：动态曝光 & SD卡日志 ======================
def adjust_exposure(img):
    """A. 根据图像亮度动态调整曝光"""
    global last_exposure_adjust_time, current_exposure_us
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_exposure_adjust_time) < EXPOSURE_ADJUST_INTERVAL:
        return
    last_exposure_adjust_time = current_time
    try:
        stats = img.get_statistics()
        brightness = stats.l_mean()
        if brightness < 40:
            current_exposure_us = min(EXPOSURE_MAX, current_exposure_us + EXPOSURE_STEP)
            sensor.set_auto_exposure(False, exposure_us=current_exposure_us)
            print(f"📷 曝光↑ -> {current_exposure_us}μs (亮度={brightness})")
        elif brightness > 200:
            current_exposure_us = max(EXPOSURE_MIN, current_exposure_us - EXPOSURE_STEP)
            sensor.set_auto_exposure(False, exposure_us=current_exposure_us)
            print(f"📷 曝光↓ -> {current_exposure_us}μs (亮度={brightness})")
    except Exception as e:
        print(f"⚠️  曝光调整失败：{e}")

def log_alarm_event(temp, fire_detected, alarm_triggered):
    """E. SD卡日志记录"""
    try:
        import uos
        try:
            uos.stat("/sd")
        except OSError:
            return
        with open(LOG_FILE, "a") as f:
            t_sec = time.ticks_ms() // 1000
            fire_flag = 1 if fire_detected else 0
            alarm_flag = 1 if alarm_triggered else 0
            f.write(f"t={t_sec},temp={temp},fire={fire_flag},alarm={alarm_flag}\n")
        print(f"💾 日志已写入")
    except Exception as e:
        print(f"⚠️  日志写入失败：{e}")

# ====================== A. 电源稳定化函数 ======================
def stabilize_power():
    """A. 系统启动时稳定电源"""
    print("⏳ 初始化系统，稳定电源...")
    time.sleep_ms(1000)
    print("✅ 电源已稳定")

# ====================== 10. 主程序 ======================
if __name__ == "__main__":
    # A. 电源稳定化
    stabilize_power()

    # 1. I2C初始化
    i2c = SoftI2C(sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=100000)
    time.sleep_ms(100)

    # 2. 扫描I2C设备
    i2c_devices = i2c.scan()
    print(f"I2C扫描到的设备地址：{[hex(d) for d in i2c_devices]}")
    if 0x38 not in i2c_devices:
        print("警告：未检测到AHT20（0x38），温湿度将显示默认值！")
    if 0x3c not in i2c_devices:
        raise Exception("错误：未检测到OLED（0x3c），请检查接线！")

    # 3. 初始化外设
    oled = SSD1306_I2C(128, 64, i2c, addr=0x3c)
    time.sleep_ms(50)
    aht20 = AHT20(i2c)
    time.sleep_ms(50)
    fire_alarm = FireAlarm(LED_STATUS, LED_ALARM, BUZZER_PIN)

    # 4. 初始化ESP8266 WiFi
    wifi_ready = False
    try:
        wifi_ready = config_esp8266_as_ap(debug=True)
    except Exception as e:
        print(f"ESP8266初始化异常：{e}")

    # OLED显示WiFi状态
    oled.fill(0)
    oled.text("Fire Detect System", 5, 10)
    oled.text("Color+Shape+Temp", 25, 30)
    oled.text("WiFi: " + ("Ready" if wifi_ready else "Failed"), 5, 45)
    oled.text("Ready!", 45, 55)
    oled.show()
    time.sleep_ms(2000)

    # 5. 摄像头初始化
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_INIT)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    clock = time.clock()

    # 6. 主循环
    prev_alarm_triggered = False
    try:
        while True:
            clock.tick()

            # 温湿度采集
            temp, hum = aht20.update_data()

            # 火焰检测 + 温度报警判断
            img = sensor.snapshot()
            fire_detected, alarm_triggered = fire_alarm.detect_fire(img, temp)

            # A. 动态曝光调整
            adjust_exposure(img)

            # WiFi通信逻辑
            if wifi_ready:
                check_esp8266_client()
                send_sensor_data_to_wifi(temp, hum, fire_detected, alarm_triggered)
                send_heartbeat()
                monitor_esp8266_health()  # E. 监控 ESP8266 健康

            # E. SD卡日志
            if alarm_triggered != prev_alarm_triggered:
                log_alarm_event(temp, fire_detected, alarm_triggered)
                prev_alarm_triggered = alarm_triggered

            # 构建报警状态文本
            if alarm_triggered:
                if fire_detected and (temp > TEMP_ALARM_THRESHOLD):
                    alarm_status = "FIRE+TEMP ALARM"
                elif fire_detected:
                    alarm_status = "FIRE ALARM"
                else:
                    alarm_status = "TEMP ALARM"
            else:
                alarm_status = "NO ALARM"

            # OLED显示
            oled.fill(0)
            oled.text("Fire&Env Monitor", 0, 0)
            temp_text = f"Temp: {temp:.1f}C"
            if temp > TEMP_ALARM_THRESHOLD:
                temp_text += " !"
            oled.text(temp_text, 0, 20)
            oled.text(f"Hum:  {hum:.1f}%RH", 0, 35)
            oled.text(alarm_status[:16], 0, 50)
            oled.show()

            # 调试打印
            print(f"\n=== 实时数据 ===")
            print(f"温度：{temp:.1f}℃ | 湿度：{hum:.1f}%RH")
            print(f"火焰检测：{'✅' if fire_detected else '❌'}")
            print(f"报警状态：{alarm_status} | 帧率：{clock.fps():.1f} FPS")
            if wifi_ready:
                print(f"WiFi客户端：{'✅' if client_connected else '❌'}")
            if temp > TEMP_ALARM_THRESHOLD:
                print(f"⚠️  警告：温度超过{TEMP_ALARM_THRESHOLD}℃！")
            if fire_detected:
                print(f"🔥 警告：检测到火焰！")

            time.sleep_ms(50)

    except KeyboardInterrupt:
        # 退出清理
        fire_alarm.led_status.value(1)
        fire_alarm.led_alarm.value(0)
        fire_alarm.buzzer_pwm.pulse_width_percent(0)
        oled.fill(0)
        oled.text("System Exit", 25, 30)
        oled.show()
        print("程序已手动退出")
