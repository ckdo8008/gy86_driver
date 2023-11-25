from hmc5883l.HMC5883L import HMC5883L
from mpu6050.MPU6050 import mpu6050
from pprint import pprint as pp
from sensor_msgs.msg import Temperature, Imu, MagneticField
import math
import time
import rclpy
from rclpy.node import Node


class IMU(Node):
    def __init__(self):
        self.__stopped = True

        self.mpu6050 = mpu6050(address=0x68)
        self.mpu6050.set_accel_range(mpu6050.ACCEL_RANGE_16G)
        self.mpu6050.set_gyro_range(mpu6050.GYRO_RANGE_2000DEG)

        self.compass = HMC5883L(address=0x1E, gauss=1.30)

        self.sensors_to_read = ('gyr', 'acc', 'comp')
        self.readings = {'gyr': (0, 0, 0),
                         'acc': (0, 0, 0),
                         'comp': (0, 0, 0)}

        self.last_read = time.time()

        self.angles = {'x': 0.0,
                       'y': 0.0,
                       'z': 0.0}
        self.orientation = (0, 0, 0, 0)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        self.mag_msg = MagneticField()

    def read_data(self, sensors_to_read=('gyr', 'acc', 'comp')):
        self.sensors_to_read = sensors_to_read

        if 'gyr' in self.sensors_to_read:
            self.readings['gyr'] = self.mpu6050.get_gyro_data()
        if 'acc' in self.sensors_to_read:
            self.readings['acc'] = self.mpu6050.get_accel_data()
        if 'comp' in self.sensors_to_read:
            self.readings['comp'] = self.compass.read_data()

        return self.readings

    def read_ros_compatible_data(self, sensors_to_read=('gyr', 'acc', 'comp')):
        s = time.time()
        _ = self.read_data(sensors_to_read)

        self.imu_msg.angular_velocity.x = self.readings['gyr'][0] * 0.0174
        self.imu_msg.angular_velocity.y = self.readings['gyr'][1] * 0.0174
        self.imu_msg.angular_velocity.z = self.readings['gyr'][2] * 0.0174
        # self.imu_msg.angular_velocity.z = 0.0

        self.imu_msg.linear_acceleration.x = self.readings['acc'][0]
        self.imu_msg.linear_acceleration.y = self.readings['acc'][1]
        self.imu_msg.linear_acceleration.z = self.readings['acc'][2]

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()

        self.mag_msg.magnetic_field.x = self.readings['comp'][0]
        self.mag_msg.magnetic_field.y = self.readings['comp'][1]
        self.mag_msg.magnetic_field.z = self.readings['comp'][2]

        self.mag_msg.header.stamp = self.get_clock().now().to_msg()

        return self.imu_msg, self.mag_msg, s

    def read_data_and_fuse(self, sensors_to_read=('gyr', 'acc', 'comp')):
        _ = self.read_data(sensors_to_read)
        dt = time.time() - self.last_read

        self.angles['x'] += self.readings['gyr'][0] * dt
        self.angles['y'] += self.readings['gyr'][1] * dt
        self.angles['z'] += self.readings['comp'][1] * dt

        if self.angles['x'] > 180.0:
            self.angles['x'] -= 360
        if self.angles['y'] > 180.0:
            self.angles['y'] -= 360

        R = (pow(self.readings['acc'][0], 2) + pow(self.readings['acc'][1], 2) + pow(self.readings['acc'][2], 2)) ** 0.5
        ax_angle = -math.atan2(self.readings['acc'][0] / R, self.readings['acc'][2] / R) * (180 / math.pi)
        ay_angle = -math.atan2(self.readings['acc'][1] / R, self.readings['acc'][2] / R) * (180 / math.pi)

        # self.angles['x'] = ax_angle
        # self.angles['y'] = ay_angle
        alpha = 0.9
        self.angles['x'] = alpha * self.angles['x'] + (1 - alpha) * ax_angle
        self.angles['y'] = alpha * self.angles['y'] + (1 - alpha) * ay_angle

        self.last_read = time.time()
        return self.angles

    @staticmethod
    def current_millis_frac():
        return time.time() * 1000
