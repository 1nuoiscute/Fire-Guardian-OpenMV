# aht20.py - DFRobot AHT20驱动适配OpenMV4（MicroPython）
from machine import I2C, Pin
import time

class DFRobot_AHT20:
    # AHT20核心参数
    AHT20_DEF_I2C_ADDR           = 0x38
    CMD_INIT                     = 0xBE
    CMD_INIT_PARAMS_1ST          = 0x08
    CMD_INIT_PARAMS_2ND          = 0x00
    CMD_INIT_TIME                = 0.01
    CMD_MEASUREMENT              = 0xAC
    CMD_MEASUREMENT_PARAMS_1ST   = 0x33
    CMD_MEASUREMENT_PARAMS_2ND   = 0x00
    CMD_MEASUREMENT_TIME         = 0.08
    CMD_MEASUREMENT_DATA_LEN     = 6
    CMD_MEASUREMENT_DATA_CRC_LEN = 7
    CMD_SOFT_RESET               = 0xBA
    CMD_SOFT_RESET_TIME          = 0.02
    CMD_STATUS                   = 0x71

    def __init__(self, i2c: I2C, addr: int = AHT20_DEF_I2C_ADDR):
        """
        初始化AHT20（适配OpenMV I2C）
        :param i2c: OpenMV的I2C对象（如I2C(1, sda=Pin('P3'), scl=Pin('P2'))）
        :param addr: I2C地址（默认0x38）
        """
        self._addr = addr
        self._i2c = i2c
        self._humidity = 0.0
        self._temperature = 0.0

    def begin(self):
        """初始化传感器，返回是否成功"""
        if self._init() != True:
            return False
        return True

    def reset(self):
        """软复位传感器"""
        self._write_command(self.CMD_SOFT_RESET)
        time.sleep(self.CMD_SOFT_RESET_TIME)

    def start_measurement_ready(self, crc_en = False):
        """启动测量并返回是否完成"""
        recv_len = self.CMD_MEASUREMENT_DATA_LEN
        if self._ready() == False:
            print("AHT20未校准！")
            return False

        if crc_en:
            recv_len = self.CMD_MEASUREMENT_DATA_CRC_LEN

        # 发送测量命令
        self._write_command_args(self.CMD_MEASUREMENT, self.CMD_MEASUREMENT_PARAMS_1ST, self.CMD_MEASUREMENT_PARAMS_2ND)
        time.sleep(self.CMD_MEASUREMENT_TIME)

        # 读取测量数据
        l_data = self._read_data(0x00, recv_len)
        if not l_data:
            print("AHT20读取数据失败！")
            return False

        # 检查忙状态
        if l_data[0] & 0x80:
            print("AHT20忙！")
            return False

        # CRC校验（若启用）
        if crc_en and self._check_crc8(l_data[6], l_data[:6]) == False:
            print("AHT20 CRC校验失败！")
            return False

        # 解析湿度数据
        humi_raw = (l_data[1] << 12) | (l_data[2] << 4) | (l_data[3] >> 4)
        self._humidity = (humi_raw / 0x100000) * 100.0

        # 解析温度数据
        temp_raw = ((l_data[3] & 0x0F) << 16) | (l_data[4] << 8) | l_data[5]
        self._temperature = (temp_raw / 0x100000) * 200.0 - 50.0
        return True

    def get_temperature_C(self):
        """获取温度（℃）"""
        return round(self._temperature, 1)

    def get_humidity_RH(self):
        """获取湿度（%RH）"""
        return round(self._humidity, 1)

    def get_temperature_F(self):
        """获取温度（℉）"""
        return round(self._temperature * 1.8 + 32, 1)

    # ---------------------- 内部函数 ----------------------
    def _check_crc8(self, crc8, data):
        """CRC8校验"""
        crc = 0xFF
        pos = 0
        size = len(data)
        while pos < size:
            crc ^= data[pos]
            i = 8
            while i > 0:
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                i -= 1
            pos += 1
        crc &= 0xFF
        return crc8 == crc

    def _ready(self):
        """检查传感器是否校准"""
        status = self._get_status_data()
        return (status & 0x08) != 0

    def _init(self):
        """传感器初始化"""
        status = self._get_status_data()
        if status & 0x08:
            return True

        self._write_command_args(self.CMD_INIT, self.CMD_INIT_PARAMS_1ST, self.CMD_INIT_PARAMS_2ND)
        time.sleep(self.CMD_INIT_TIME)

        status = self._get_status_data()
        return (status & 0x08) != 0

    def _get_status_data(self):
        """读取状态寄存器"""
        status = self._read_data(self.CMD_STATUS, 1)
        return status[0] if status else 0

    def _read_data(self, cmd, len):
        """读取I2C数据（OpenMV适配）"""
        try:
            return self._i2c.readfrom_mem(self._addr, cmd, len)
        except:
            return []

    def _write_command(self, cmd):
        """发送单字节命令"""
        try:
            self._i2c.writeto(self._addr, bytes([cmd]))
        except:
            pass

    def _write_command_args(self, cmd, args1, args2):
        """发送带参数的命令"""
        try:
            self._i2c.writeto(self._addr, bytes([cmd, args1, args2]))
        except:
            pass
