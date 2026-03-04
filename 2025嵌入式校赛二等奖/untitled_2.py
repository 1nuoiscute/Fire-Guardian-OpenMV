import time
import framebuf
import sensor
import image
import math
import json
from machine import I2C, Pin
from machine import UART
from pyb import Timer
from micropython import const

# ====================== 1. 硬件引脚定义 ======================
I2C_SDA_PIN = 'P5'
I2C_SCL_PIN = 'P4'
LED_STATUS = 'P3'   # 正常常亮，报警时熄灭
LED_ALARM = 'P2'    # 正常熄灭，报警时亮起
BUZZER_PIN = 'P7'   # 正常静音，报警时鸣响
uart = UART(1, 115200)
# ====================== 2. 核心检测配置 ======================
# 基于直方图的LAB颜色阈值
FIRE_LAB_THRESHOLD =(29, 100, -29, 46, 15, 60) #(11, 100, -1, 11, 2, 12)
# 颜色检测灵敏度
FIRE_PIXEL_THRESHOLD = 10
FIRE_AREA_THRESHOLD = 15
FIRE_MERGE = True
FIRE_DETECT_INTERVAL = 80

# 火焰形态特征阈值
MIN_ASPECT_RATIO = 1.0    # 椭圆轮廓长宽比下限（椭圆h/w≈1~2.5）
MAX_ASPECT_RATIO = 2.5    # 椭圆轮廓长宽比上限
MAX_CIRCULARITY = 0.65    # 椭圆轮廓圆形度（<0.65，排除正圆；>0.3保留椭圆）
MIN_CIRCULARITY = 0.3     # 新增：排除过度不规则的杂点
MIN_EDGE_RATIO = 0.4      # 边沿像素占比下限（边沿区域>40%，排除中间实心类圆）
MAX_CORE_DENSITY = 0.4    # 核心密度上限（中间高亮区>0.4，直接排除）
#  温度报警阈值
TEMP_ALARM_THRESHOLD = 50.0  # 温度超过50℃触发报警
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

# ====================== 4. CRC8校验函数 ======================
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

# ====================== 5. AHT20驱动（容错版） ======================
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

# ====================== 6. SSD1306 OLED驱动 ======================
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

# ====================== 7. 火焰&温度报警模块 ======================
class FireAlarm:
    def __init__(self, led_status_pin, led_alarm_pin, buzzer_pin):
        # 初始化硬件外设
        self.led_status = Pin(led_status_pin, Pin.OUT, value=1)
        self.led_alarm = Pin(led_alarm_pin, Pin.OUT, value=0)
        self.tim = Timer(4, freq=2000)
        self.buzzer_pwm = self.tim.channel(1, Timer.PWM, pin=Pin(buzzer_pin))
        self.buzzer_pwm.pulse_width_percent(0)

        # 火焰检测参数
        self.fire_threshold = FIRE_LAB_THRESHOLD
        self.last_detect_time = 0
        self.detect_interval = FIRE_DETECT_INTERVAL

    # 新增：计算色块圆形度（近似值，OpenMV无直接周长接口）
    def _calc_circularity(self, blob):
        # 圆形度公式：4π*像素数 / (外框周长²)，越接近1越圆
        pixels = blob.pixels()
        w = blob.w()
        h = blob.h()
        perimeter = 2 * (w + h)  # 外框周长（近似）
        if perimeter == 0:
            return 0.0
        circularity = (4 * math.pi * pixels) / (perimeter * perimeter)
        return circularity

    # 火焰形态判断
    def _is_fire_shape(self, blob):
        # 1. 计算长宽比（h/w）
        w = blob.w()
        h = blob.h()
        if w == 0:
            return False
        aspect_ratio = h / w

        # 2. 计算圆形度
        circularity = self._calc_circularity(blob)

        # 3. 形态特征判断：长宽比在0.5~3之间 + 圆形度<0.7
        if (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO) and (circularity < MAX_CIRCULARITY):
            return True
        return False

    def detect_fire(self, img, temp):
        current_time = time.ticks_ms()
        fire_detected = False

        # 火焰检测逻辑（颜色+形态）
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
            # 颜色+形态双过滤
            for b in blobs:
                # 第一步：颜色过滤（密度>0.4）
                if b.density() > 0.4:
                    # 第二步：形态过滤（火焰不规则形态）
                    if self._is_fire_shape(b):
                        valid_fire_blobs.append(b)
                        # 绘制识别框+十字（可视化）
                        img.draw_rectangle(b.rect(), color=(255, 0, 0), thickness=2)
                        img.draw_cross(b.cx(), b.cy(), color=(0, 255, 0), size=4)
                        # 绘制形态特征（调试用）
                        img.draw_string(b.x(), b.y()-10, f"AR:{b.h()/b.w():.1f}", color=(255,255,0))
                        img.draw_string(b.x(), b.y()-20, f"C:{self._calc_circularity(b):.1f}", color=(255,255,0))

            # 判定火焰状态：同时满足颜色+形态
            fire_detected = len(valid_fire_blobs) > 0
            self.last_detect_time = current_time

        # 复合报警条件：火焰检测到 OR 温度超过50℃
        alarm_triggered = fire_detected or (temp > TEMP_ALARM_THRESHOLD)

        # 外设控制
        if alarm_triggered:
            self.led_status.value(0)    # 状态灯熄灭
            self.led_alarm.value(1)     # 报警灯亮起
            self.buzzer_pwm.pulse_width_percent(80)  # 蜂鸣器鸣响
        else:
            self.led_status.value(1)    # 状态灯常亮
            self.led_alarm.value(0)     # 报警灯熄灭
            self.buzzer_pwm.pulse_width_percent(0)   # 蜂鸣器静音

        return fire_detected, alarm_triggered

# ====================== 8. 主程序 ======================
if __name__ == "__main__":
    # 1. I2C初始化
    i2c = I2C(sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=100000)
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
    # ESP8266 AT指令 - 配置AP模式
    at_set_ap_mode = "AT+CWMDOE=2"

    # ESP8266 AT指令 - 配置AP热点参数（SSID、密码、信道、加密方式）
    at_config_ap_params = "AT+CWSAP=\"ESP8266\",\"12345678\",11,3"

    # ESP8266 AT指令 - 建立TCP连接（目标IP+端口）
    at_create_tcp_connection = "AT+CIPSTART=\"TCP\",\"192.168.4.2\",8888"

    # ESP8266 AT指令 - 指定发送数据长度（5字节）
    at_set_send_data_length = "AT+CIPSEND=5"
    uart.write(at_set_ap_mode)
    uart.write( at_config_ap_params)
    uart.write(at_create_tcp_connection)
    uart.write(at_create_tcp_connection)

    # 4. 摄像头初始化
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_auto_exposure(False, exposure_us=5000)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    clock = time.clock()

    # 5. OLED初始化显示
    oled.fill(0)
    oled.text("Fire Detect System", 5, 10)
    oled.text("Color+Shape+Temp", 25, 30)
    oled.text("Ready!", 45, 50)
    oled.show()
    time.sleep_ms(2000)

    # 6. 主循环
    try:
        while True:
            clock.tick()

            # 温湿度采集
            temp, hum = aht20.update_data()

            # 火焰检测（颜色+形态）+ 温度报警判断
            img = sensor.snapshot()
            fire_detected, alarm_triggered = fire_alarm.detect_fire(img, temp)

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

            # OLED显示（优化温度超限提示）
            oled.fill(0)
            oled.text("Fire&Env Monitor", 0, 0)
            # 温度显示：超限标注警告符号
            temp_text = f"Temp: {temp:.1f}C"
            if temp > TEMP_ALARM_THRESHOLD:
                temp_text += " !"
            oled.text(temp_text, 0, 20)
            oled.text(f"Hum:  {hum:.1f}%RH", 0, 35)
            oled.text(f"Status: {alarm_status[:12]}", 0, 50)  # 截断防止超出屏幕
            oled.show()
            json_array = json.dumps([temp, hum])
            uart.write(json_array+'\n')
            # 调试打印
            print(f"\n=== 实时数据 ===")
            print(f"温度：{temp:.1f}℃ | 湿度：{hum:.1f}%RH")
            print(f"报警状态：{alarm_status} | 帧率：{clock.fps():.1f} FPS")
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
