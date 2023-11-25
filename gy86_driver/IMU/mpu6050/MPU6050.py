from __future__ import print_function
import smbus
import math
import rclpy

class mpu6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    IMU_CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    USER_CTRL = 0x6A
    INT_PIN_CFG = 0x37

    def __init__(self, address, bus=0):
        self.address = address
        self.bus = smbus.SMBus(bus)

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(self.address, self.IMU_CONFIG, 0x00)
        self.bypass_mode_en()

        self.gyroscope_offsets = {'gx': 0.0,
                                  'gy': 0.0,
                                  'gz': 0.0}
        self.accelerometer_offsets = {'ax': 0.0,
                                      'ay': 0.0,
                                      'az': 0.0}

        self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def read_i2c_xyz_block(self, register):
        """Read six i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        # high = self.bus.read_byte_data(self.address, register)
        # low = self.bus.read_byte_data(self.address, register + 1)
        high_x, low_x, high_y, low_y, high_z, low_z = self.bus.read_i2c_block_data(self.address, register, 6)

        value_x = (high_x << 8) + low_x
        value_y = (high_y << 8) + low_y
        value_z = (high_z << 8) + low_z

        if (value_x >= 0x8000):
            value_x = -((65535 - value_x) + 1)
        if (value_y >= 0x8000):
            value_y = -((65535 - value_y) + 1)
        if (value_z >= 0x8000):
            value_z = -((65535 - value_z) + 1)

        return value_x, value_y, value_z

    # MPU-6050 Methods

    def bypass_mode_en(self):
        """
        enable the bypass mode of MPU6050 for the HMC5881 can be found by the host processor
        When I2C_BYPASS_EN is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN.
        USER_CTRL = 0x6A
        INT_PIN_CFG = 0x37
        """

        usr_ctrl = self.bus.read_byte_data(self.address, self.USER_CTRL)
        print('USER_CTRL= ', usr_ctrl)
        usr_ctrl = usr_ctrl | 0x02
        self.bus.write_byte_data(self.address, self.USER_CTRL, usr_ctrl)
        print('USER_CTRL= ', usr_ctrl)

        pin_cfg = self.bus.read_byte_data(self.address, self.INT_PIN_CFG)
        print('INT_PIN_CFG= ', pin_cfg)
        pin_cfg = pin_cfg | 0x02
        self.bus.write_byte_data(self.address, self.INT_PIN_CFG, pin_cfg)
        print('INT_PIN_CFG= ', pin_cfg)

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

        if gyro_range == self.GYRO_RANGE_250DEG:
            self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            self.gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG


    def read_gyro_range(self, raw=False):
        """Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.
        Returns the read values in a dictionary.
        """
        x, y, z = self.read_i2c_xyz_block(self.GYRO_XOUT0)

        x = (x / self.gyro_scale_modifier) - self.gyroscope_offsets['gx']
        y = (y / self.gyro_scale_modifier) - self.gyroscope_offsets['gy']
        z = (z / self.gyro_scale_modifier) - self.gyroscope_offsets['gz']

        return x, y, z

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

        if accel_range == self.ACCEL_RANGE_2G:
            self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            self.accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

    def read_accel_range(self, raw=False):
        """Reads the range the accelerometer is set to.
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self):
        """Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x, y, z = self.read_i2c_xyz_block(self.ACCEL_XOUT0)

        x = (x / self.accel_scale_modifier) * self.GRAVITIY_MS2 - self.accelerometer_offsets['ax']
        y = (y / self.accel_scale_modifier) * self.GRAVITIY_MS2 - self.accelerometer_offsets['ay']
        z = (z / self.accel_scale_modifier) * self.GRAVITIY_MS2 - self.accelerometer_offsets['az']

        return x, y, z

    def _equal(self, value, reference, error_margin=0.1):
        return (reference - error_margin) <= value <= (reference + error_margin)

    def calibrate_gyroscope(self, num_of_calibration_measurements=4500, print_status=False):

        calibration_cycle_count = 0
        total_gyr_offset = [0, 0, 0]
        max_line_length = 0
        self.gyroscope_offsets = {'gx': 0.0,
                                  'gy': 0.0,
                                  'gz': 0.0}

        if print_status:
            print("Calibrating gyroscope offsets, please hold still...")

        while (calibration_cycle_count < num_of_calibration_measurements) and (not rclpy.ok()):

            (gx, gy, gz) = self.get_gyro_data()
            total_gyr_offset[0] += gx
            total_gyr_offset[1] += gy
            total_gyr_offset[2] += gz

            calibration_cycle_count += 1

            if print_status:
                avg_gx = total_gyr_offset[0] / calibration_cycle_count
                avg_gy = total_gyr_offset[1] / calibration_cycle_count
                avg_gz = total_gyr_offset[2] / calibration_cycle_count

                s = "[{}/{}] gx:{:0.4f}  gy:{:0.4f}  gz:{:0.4f}".format(calibration_cycle_count,
                                                                        num_of_calibration_measurements,
                                                                        avg_gx,
                                                                        avg_gy,
                                                                        avg_gz)

                max_line_length = len(s) if len(s) > max_line_length else max_line_length
                num_of_space = max_line_length - len(s)

                print(s + ' ' * num_of_space, end='\r')

        if print_status:
            print('\nDone.')

        self.gyroscope_offsets = {'gx': total_gyr_offset[0] / calibration_cycle_count,
                                  'gy': total_gyr_offset[1] / calibration_cycle_count,
                                  'gz': total_gyr_offset[2] / calibration_cycle_count}

        return not rclpy.ok()

    def calibrate_accelerometer(self, num_of_calibration_measurements=4500, print_status=False):
        """ Auto calibrate the device offset. Put the device so as one axe is parallel to the gravity field (usually, put the device on a flat surface) """

        err_count = 0
        max_line_length = 0
        calibration_cycle_count = 0
        total_acc_offset = [0, 0, 0]
        self.accelerometer_offsets = {'ax': 0.0,
                                      'ay': 0.0,
                                      'az': 0.0}

        if print_status:
            print("Calibrating accelerometer offsets, please hold still...")

        while (calibration_cycle_count < num_of_calibration_measurements) and (not rclpy.ok()):
            ax, ay, az = self.get_accel_data()
            calibration_cycle_count += 1

            abs_x = math.fabs(ax)
            abs_y = math.fabs(ay)
            abs_z = math.fabs(az)

            # Find which axe is in the field of gravity and set its expected value to 1g absolute value
            if self._equal(abs_x, 1) and self._equal(abs_y, 0) and self._equal(abs_z, 0):
                cal_x = 1 if ax > 0 else -1
                cal_y = 0
                cal_z = 0
            elif self._equal(abs_x, 0) and self._equal(abs_y, 1) and self._equal(abs_z, 0):
                cal_x = 0
                cal_y = 1 if ay > 0 else -1
                cal_z = 0
            elif self._equal(abs_x, 0) and self._equal(abs_y, 0) and self._equal(abs_z, 1):
                cal_x = 0
                cal_y = 0
                cal_z = 1 if az > 0 else -1
            else:
                err_count += 1
                num_of_calibration_measurements += 10
                cal_x = 0
                cal_y = 0
                cal_z = 0

            total_acc_offset[0] += cal_x - ax
            total_acc_offset[1] += cal_y - ay
            total_acc_offset[2] += cal_z - az

            if print_status:
                avg_ax = total_acc_offset[0] / calibration_cycle_count
                avg_ay = total_acc_offset[1] / calibration_cycle_count
                avg_az = total_acc_offset[2] / calibration_cycle_count

                s = "[{}/{}][Error Count:{}] ax:{:0.4f}  ay:{:0.4f}  az:{:0.4f}".format(calibration_cycle_count,
                                                                                        num_of_calibration_measurements,
                                                                                        err_count,
                                                                                        avg_ax,
                                                                                        avg_ay,
                                                                                        avg_az)

                max_line_length = len(s) if len(s) > max_line_length else max_line_length
                num_of_space = max_line_length - len(s)

                print(s + ' ' * num_of_space, end='\r')

        if print_status:
            print('\nDone.')

        self.accelerometer_offsets = {'ax': total_acc_offset[0] / calibration_cycle_count,
                                      'ay': total_acc_offset[1] / calibration_cycle_count,
                                      'az': total_acc_offset[2] / calibration_cycle_count}

        return not rclpy.ok()


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    print(mpu.get_temp())
    accel_data = mpu.get_accel_data()
    print(accel_data[0])
    print(accel_data[1])
    print(accel_data[2])
    gyro_data = mpu.get_gyro_data()
    print(gyro_data[0])
    print(gyro_data[1])
    print(gyro_data[2])
