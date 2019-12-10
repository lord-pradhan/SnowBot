"""
Program:     ICM20948.py
Revised On:  12/07/2019
"""

### Library Imports
import pigpio
from time import sleep
###


### Class Definition
class ICM20948:
    
    # ICM-20948 Registers
    REG_REG_BANK_SEL = 127
    
    # User Bank 0
    REG0_WHO_AM_I = 0
    REG0_USER_CTRL = 3
    REG0_LP_CONFIG = 5
    REG0_PWR_MGMT_1 = 6
    REG0_PWR_MGMT_2 = 7
    REG0_INT_PIN_CFG = 15
##    REG0_INT_ENABLE = 16
##    REG0_INT_ENABLE_1 = 17
##    REG0_INT_ENABLE_2 = 18
##    REG0_INT_ENABLE_3 = 19
##    REG0_INT_STATUS = 25
    REG0_INT_STATUS_1 = 26
##    REG0_INT_STATUS_2 = 27
##    REG0_INT_STATUS_3 = 28
##    REG0_DELAY_TIMEH = 40
##    REG0_DELAY_TIMEL = 41
    REG0_ACCEL_XOUT_H = 45
    REG0_ACCEL_XOUT_L = 46
    REG0_ACCEL_YOUT_H = 47
    REG0_ACCEL_YOUT_L = 48
    REG0_ACCEL_ZOUT_H = 49
    REG0_ACCEL_ZOUT_L = 50
    REG0_GYRO_XOUT_H = 51
    REG0_GYRO_XOUT_L = 52
    REG0_GYRO_YOUT_H = 53
    REG0_GYRO_YOUT_L = 54
    REG0_GYRO_ZOUT_H = 55
    REG0_GYRO_ZOUT_L = 56
##    REG0_FIFO_EN_1 = 102
##    REG0_FIFO_EN_2 = 103
##    REG0_FIFO_RST = 104
##    REG0_FIFO_MODE = 105
##    REG0_FIFO_COUNTH = 112
##    REG0_FIFO_COUNTL = 113
##    REG0_FIFO_R_W = 114
##    REG0_DATA_RDY_STATUS = 116
##    REG0_FIFO_CFG = 118
    
    # User Bank 1
    REG1_XA_OFFS_H = 20
    REG1_XA_OFFS_L = 21
    REG1_YA_OFFS_H = 22
    REG1_YA_OFFS_L = 23
    REG1_ZA_OFFS_H = 24
    REG1_ZA_OFFS_L = 25
    REG1_TIMEBASE_CORRECTION_PLL = 40
    
    # User Bank 2
    REG2_GYRO_SMPLRT_DIV = 0
    REG2_GYRO_CONFIG_1 = 1
    REG2_GYRO_CONFIG_2 = 2
    REG2_XG_OFFS_USRH = 3
    REG2_XG_OFFS_USRL = 4
    REG2_YG_OFFS_USRH = 5
    REG2_YG_OFFS_USRL = 6
    REG2_ZG_OFFS_USRH = 7
    REG2_ZG_OFFS_USRL = 8
    REG2_ODR_ALIGN_EN = 9
    REG2_ACCEL_SMPLRT_DIV_1 = 16
    REG2_ACCEL_SMPLRT_DIV_2 = 17
    REG2_ACCEL_INTEL_CTRL = 18
    REG2_ACCEL_WOM_THR = 19
    REG2_ACCEL_CONFIG = 20
    REG2_ACCEL_CONFIG_2 = 21
##    REG2_FSYNC_CONFIG = 82
##    REG2_MOD_CTRL_USR = 84
    
    # Magnetometer
    REGM_WIA2 = 0X01
    REGM_ST1 = 0X10
    REGM_HXL = 0X11
    REGM_HXH = 0X12
    REGM_HYL = 0X13
    REGM_HYH = 0X14
    REGM_HZL = 0X15
    REGM_HZH = 0X16
    REGM_ST2 = 0X18
    REGM_CNTL2 = 0X31
    REGM_CNTL3 = 0X32
    
    # I2C 7-bit base address
    ADDR_AG_BASE = 0b1101000
    ADDR_M = 0x0c
    
    # Default Bytes
    WHO_AM_I = 0xea
    
    def __init__(self, i2c_channel=1, ad0_bit=1):
        self.ch = i2c_channel
        self.addr_ag = self.ADDR_AC_BASE if not ad0_bit else (self.ADDR_AG_BASE | 1)
        self.addr_m = self.ADDR_M
        
        (self.pi, self.handle_ag, self.handle_m) = self.__open_i2c()
        
        self.bank = 0
        self.accel_fsr = 0.0
        self.gyro_fsr = 0.0
        self.mag_sens = 0.15
        
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.gyroDLPFRegByte = 0x00
        self.gyroFSRegByte = 0x00
        self.accelDLPFRegByte = 0x00
        self.accelFSRegByte = 0x00

        self.yaw = theta_init
        self.sum_yaw_rate = 0
        self.ct_yaw = 0
        self.time = int(round(time.time()))
        self.yawZero = 0.018# Need to change, from old IMU
    
    def __open_i2c(self):
        pi = pigpio.pi()
        try:
            handle_ag = pi.i2c_open(self.ch, self.addr_ag)
            handle_m = pi.i2c_open(self.ch, self.addr_m)
            return (pi, handle_ag, handle_m)
        except:
            print('I2C open failed.')
            return (-1, -1)
    
    def __write_byte(self, handle, reg, data):
        while(1):
            try:
                self.pi.i2c_write_byte_data(handle, reg, data)
                break
            except:
                print('i2c write IMU try-catch')
                continue
    
    def __read_byte(self, handle, reg):
        while(1):
            try:
                return self.pi.i2c_read_byte_data(handle, reg)
            except:
                print('i2c read IMU try-catch')
                continue
    
    def __read(self, handle, reg, numBytes):
        while(1):
            try:
                (num_bytes, data_bytearray) = self.pi.i2c_read_i2c_block_data(handle, reg, numBytes)
                return list(data_bytearray)
            except:
                print('i2c read (multi) IMU try-catch')
                continue
    
    def __set_bank(self, bank):
        """Set the User Bank"""
        if(bank != self.bank):
            self.__write_byte(self.handle_ag, self.REG_REG_BANK_SEL, bank)
            self.bank = bank

    def setup(self):
        self.accel_gyro_soft_reset()
        self.initialize()
        self.set_accel_sample_rate(100)
        self.set_accel_digital_filter(5, enable=1)
        self.set_accel_range(0)
        self.set_gyro_sample_rate(100)
        self.set_gyro_digital_filter(5, enable=1)
        self.set_gyro_range(0)

        self.mag_soft_reset()
        self.set_mag_sample_rate(0b01000)
    
    def initialize(self):
        # Set to User Bank 0
        self.__set_bank(0)
        # Wake IMU, disable sleep, disable cycling, disable temperature sensor,
        # auto-select oscillator, 
        self.__write_byte(self.handle_ag, self.REG0_PWR_MGMT_1, 0b00001001)
        # Enable accelerometer, enable gyroscope
##        self.__write_byte(self.handle_ag, self.REG0_PWR_MGMT_2, 0b00000000)
        # Enable I2C master cycling, disable accelerometer & gyro cycling.
##        self.__write_byte(self.handle_ag, self.REG0_LP_CONFIG, 0b01000000)
        # Disable DMP, disable FIFO, disable I2C Master
##        self.__write_byte(self.handle_ag, self.REG0_USER_CTRL, 0b00000000)
        # Set I2C Master bypass
        self.__write_byte(self.handle_ag, self.REG0_INT_PIN_CFG, 0b00000010)

    def accel_gyro_soft_reset(self):
        # Set to User Bank 0
        self.__set_bank(0)
        # Reset IMU.
        self.__write_byte(self.handle_ag, self.REG0_PWR_MGMT_1, 0b10000000)
        sleep(0.1)

    def set_gyro_sample_rate(self, rate):
        """Set the gyroscope sample rate divider.
        
        Args:
            rate - Desired sampling rate, in Hz.
        Returns:
            float - Actual sampling rate, in Hz.
        """
        data = int(1100 / rate - 1) & 0xff
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_GYRO_SMPLRT_DIV, data)
        return 1100 / float(data + 1)
    
    def set_gyro_digital_filter(self, config, enable=1):
        """Configure digital low-pass filter for gyroscope.
        
        Args:
            config - DLPF configuration value. Valid values:
                X - 12106   Hz
                0 -   196.6 Hz
                1 -   151.8 Hz
                2 -   119.5 Hz
                3 -    51.2 Hz
                4 -    23.9 Hz
                5 -    11.6 Hz
                6 -     5.7 Hz
                7 -   361.4 Hz
        """
        self.gyroDLPFRegByte = ((config & 0x07) << 3) | (enable & 0x01)
        data = self.gyroDLPFRegByte  | self.gyroFSRegByte
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_GYRO_CONFIG_1, data)

    def set_gyro_range(self, fsr):
        """Set the gyroscope full-scale range.
        
        Args:
            fsr - Full-scale range configuration value. Valid values:
                0 - +- 250 deg/s
                1 - +- 500 deg/s
                2 - +-1000 deg/s
                3 - +-2000 deg/s
        """
        self.gyro_fsr = fsr

        self.gyroFSRegByte = (fsr & 0x03) << 1
        data = self.gyroFSRegByte | self.gyroDLPFRegByte
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_GYRO_CONFIG_1, data)

    def set_accel_sample_rate(self, rate):
        """Set the accelerometer sample rate divider.
        
        Args:
            rate - Desired sampling rate, in Hz.
        Returns:
            float - Actual sampling rate, in Hz.
        """
        data = int(1125 / rate - 1) & 0x0fff
        data1 = data >> 8
        data2 = data & 0x00ff
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_ACCEL_SMPLRT_DIV_1, data1)
        self.__write_byte(self.handle_ag, self.REG2_ACCEL_SMPLRT_DIV_1, data1)
        return 1125 / float(data + 1)
    
    def set_accel_digital_filter(self, config, enable=1):
        """Configure digital low-pass filter for accelerometer.
        
        Args:
            config - DLPF configuration value. Valid values:
                X - 1209   Hz
                0 -  246.0 Hz
                1 -  246.0 Hz
                2 -  111.4 Hz
                3 -   50.4 Hz
                4 -   23.9 Hz
                5 -   11.5 Hz
                6 -    5.7 Hz
                7 -  473   Hz
        """
        self.accelDLPFRegByte = ((config & 0x07) << 3) | (enable & 0x01)
        data = self.accelDLPFRegByte | self.accelFSRegByte
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_ACCEL_CONFIG, data)

    def set_accel_range(self, fsr):
        """Set the accelerometer full-scale range.
        
        Args:
            fsr - Full-scale range configuration value. Valid values:
                0 - +- 2 g
                1 - +- 4 g
                2 - +- 8 g
                3 - +-16 g
        """
        self.accel_fsr = fsr

        self.accelFSRegByte = (fsr & 0x03) << 1
        data = self.accelFSRegByte | self.accelDLPFRegByte
        self.__set_bank(2)
        self.__write_byte(self.handle_ag, self.REG2_ACCEL_CONFIG, data)

    def mag_soft_reset(self):
        self.__write_byte(self.handle_m, self.REGM_CNTL3, 0b00000001)
    
    def set_mag_sample_rate(self, rateMode):
        """Set the magnetometer sample rate mode.
        
        Args:
            rateMode - Desired sampling rate, in Hz.
                0  - Power-down
                1  - Single measurement
                2  - Continuous 1 ( 10 Hz)
                4  - Continuous 2 ( 20 Hz)
                6  - Continuous 3 ( 50 Hz)
                8  - Continuous 4 (100 Hz)
                16 - Self-test
        """
        data = rateMode & 0x1f
        if((data != 0) or (data != 1) or (data != 2) or (data != 4) or (data != 8) or (data != 16)):
            data = 2
        self.__write_byte(self.handle_m, self.REGM_CNTL2, data)
    
    def update_all(self):
        """Read most recent sensor data."""
        self.__set_bank(0)
        ag_data = self.__read(self.handle_ag, self.REG0_ACCEL_XOUT_H, 12)
        m_data = self.__read(self.handle_m, self.REGM_WIA2, 23)

        self.accel_x = self.__accel_word_to_float(ag_data[0:2])
        self.accel_y = self.__accel_word_to_float(ag_data[2:4])
        self.accel_z = self.__accel_word_to_float(ag_data[4:6])
        self.gyro_x = self.__gyro_word_to_float(ag_data[6:8])
        self.gyro_y = self.__gyro_word_to_float(ag_data[8:10])
        self.gyro_z = self.__gyro_word_to_float(ag_data[10:])
        self.mag_x = self.__mag_word_to_float(m_data[0x11:0x13])
        self.mag_y = self.__mag_word_to_float(m_data[0x13:0x15])
        self.mag_z = self.__mag_word_to_float(m_data[0x15:0x17])
    
    def __accel_word_to_float(self, arr):
        """Converts the accelerometer 16-bit 2's-complement ADC word into appropriate float.    
        
        Args:
            arr - 16-bit 2's-complement ADC word (as length-2 array).
        Return:
            float
        """
        word = (arr[0] << 8) | arr[1]
        if word & 0x8000:
            word = -1 * ((word ^ 0xffff) + 1)
        if self.accel_fsr == 0:
            return float(word) / 16384.0
        elif self.accel_fsr == 1:
            return float(word) / 8192.0
        elif self.accel_fsr == 2:
            return float(word) / 4096.0
        else:
            return float(word) / 2048.0
    
    def __gyro_word_to_float(self, arr):
        """Converts the gyroscope 16-bit 2s-complement ADC word into appropriate float.
        
        Args:
            arr - 16-bit 2's-complement ADC word (as length-2 array).
        Return:
            float
        """
        word = (arr[0] << 8) | arr[1]
        if word & 0x8000:
            word = -1 * ((word ^ 0xffff) + 1)
        if self.gyro_fsr == 0:
            return float(word) / 131.0
        elif self.gyro_fsr == 1:
            return float(word) / 65.5
        elif self.gyro_fsr == 2:
            return float(word) / 32.8
        else:
            return float(word) / 16.4

    def __mag_word_to_float(self, arr):
        """Converts the magnetometer 16-bit 2s-complement ADC word into appropriate float.
        
        Args:
            arr - 16-bit 2's-complement ADC word (as length-2 array).
        Return:
            float
        """
        word = (arr[1] << 8) | arr[0]
        if word & 0x8000:
            word = -1 * ((word ^ 0xffff) + 1)
        return float(word) * self.mag_sens
    
    def test_connection(self):
        data = self.__read_byte(self.handle_ag, self.REG0_WHO_AM_I)
        if data == self.WHO_AM_I:
            return 0
        else:
            return -1
    
    def close(self):
        self.pi.i2c_close(self.handle_ag)
        self.pi.i2c_close(self.handle_m)
        self.pi.stop()

    def getYaw(self):
        # From MPU6050 class, needs updating"""
        self.update_all()
        yaw_rate = np.radians(self.gyro_z)
        self.sum_yaw_rate += yaw_rate
        self.ct_yaw += 1
        self.YRav = self.sum_yaw_rate /  self.ct_yaw
##        print('IMU charac', yaw_rate, self.YRav)
        currTime = int(round(time.time()))
        elapsedTime = ( currTime - self.time )
        self.time = currTime
        new_yaw = self.yaw + (yaw_rate+self.yawZero)*elapsedTime
        self.yaw = wrap2pi(new_yaw)

        return self.yaw
###
