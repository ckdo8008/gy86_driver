#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from numpy import matrix
from IMU.IMU import IMU

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu = IMU()

        # 파라미터 설정
        self.imu.mpu6050.gyroscope_offsets = {
            'gx': self.get_parameter("gyroscope_offsets.gx").value,
            'gy': self.get_parameter("/imu_node/gyroscope_offsets/gy").value,
            'gz': self.get_parameter("/imu_node/gyroscope_offsets/gz").value
            # ...
        }
        # 나머지 파라미터도 유사하게 설정
        # ...

        m = self.get_parameter("compass_calibration_matrix").value
        if len(m) != 9: raise Exception("Calibration Matrix is not 3 x 3")
        self.imu.compass.calibration_matrix = matrix([m[0:3], m[3:6], m[6:9]])

        # 메시지 발행자 설정
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # 타이머 설정
        self.timer = self.create_timer(0.005, self.publish_imu)

    def publish_imu(self):
        imu_msg, mag_msg, _ = self.imu.read_ros_compatible_data(sensors_to_read=('gyr', 'acc', 'comp'))
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

if __name__ == '__main__':
    rclpy.init()
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    rclpy.shutdown()
