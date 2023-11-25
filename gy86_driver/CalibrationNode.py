#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from IMU import IMU
import yaml
import datetime
from rclpy.duration import Duration

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration_node')
        self.imu = IMU()
        self.timestamp = datetime.datetime.now()

        # 파라미터 읽기
        self.gyr_offsets = {
            'gyroscope_offsets': {
                'gx': self.get_parameter("/gyroscope_offsets/gx").value,
                'gy': self.get_parameter("/gyroscope_offsets/gy").value,
                'gz': self.get_parameter("/gyroscope_offsets/gz").value
            }
        }
        self.acc_offsets = {
            'accelerometer_offsets': {
                'ax': self.get_parameter("/accelerometer_offsets/ax").value,
                'ay': self.get_parameter("/accelerometer_offsets/ay").value,
                'az': self.get_parameter("/accelerometer_offsets/az").value
            }
        }
        self.comp_offsets = {
            'compass_offsets': {
                'cx': self.get_parameter("/compass_offsets/cx").value,
                'cy': self.get_parameter("/compass_offsets/cy").value,
                'cz': self.get_parameter("/compass_offsets/cz").value
            }
        }
        self.comp_matrix = {
            'compass_calibration_matrix': self.get_parameter("/compass_calibration_matrix").value}
        self.success = False

    def run_calibration(self):
        # 기존 스크립트의 캘리브레이션 로직
        # ...
        
        if self.get_parameter("~calibrate_gyr").value and (not rclpy.ok()):
            success = self.imu.mpu6050.calibrate_gyroscope(self.get_parameter("~gyr_num_of_calibration_measurements").value,
                                                    self.get_parameter("~print_gyr_status").value)
            if success:
                self.gyr_offsets['gyroscope_offsets'] = self.imu.mpu6050.gyroscope_offsets
        else:
            print("Skipping gyroscope calibration...")

        # Accelerometer calibration
        success = False
        if self.get_parameter("~calibrate_acc").value and (not rclpy.ok()):
            sleep_duration = Duration(seconds=2)
            rclpy.sleep(sleep_duration)
            success = self.imu.mpu6050.calibrate_accelerometer(self.get_parameter("~acc_num_of_calibration_measurements").value,
                                                        self.get_parameter("~print_acc_status").value)
            if success:
                self.acc_offsets['accelerometer_offsets'] = self.imu.mpu6050.accelerometer_offsets
        else:
            print("Skipping accelerometer calibration...")

        # Compass calibration
        success = False
        if self.get_parameter("~calibrate_comp").value and (not rclpy.ok()):
            sleep_duration = Duration(seconds=2)
            rclpy.sleep(sleep_duration)
            success = self.imu.compass.calibrate(self.get_parameter("~comp_num_of_calibration_measurements").value,
                                            self.get_parameter("~print_comp_status").value,
                                            self.timestamp)
            if success:
                self.comp_offsets['compass_offsets'] = self.imu.compass.offsets
                m = self.imu.compass.calibration_matrix
                print(m)
                self.comp_matrix['compass_calibration_matrix'] = [round(float(m[0, 0]), 5), round(float(m[0, 1]), 5), round(float(m[0, 2]), 5),
                                                            round(float(m[1, 0]), 5), round(float(m[1, 1]), 5), round(float(m[1, 2]), 5),
                                                            round(float(m[2, 0]), 5), round(float(m[2, 1]), 5), round(float(m[2, 2]), 5)]
        else:
            print("Skipping compass calibration...")

        s = ''
        s += yaml.dump(self.gyr_offsets, default_flow_style=False)
        s += yaml.dump(self.acc_offsets, default_flow_style=False)
        s += yaml.dump(self.comp_offsets, default_flow_style=False)
        s += yaml.dump(self.comp_matrix, width=25)
        print(s)

        new_filename = '/home/nano/catkin_ws/src/gy-86/imu_calibration/yaml_files/{}-imu_calibration.yaml'.format(self.timestamp)
        with open(new_filename, 'w') as the_new_yaml_file:
            the_new_yaml_file.write(s)

        old_filename = '/home/nano/catkin_ws/src/gy-86/imu_calibration/yaml_files/imu_calibration.yaml'
        with open(old_filename, 'w') as the_old_yaml_file:
            the_old_yaml_file.write(s)            
        
if __name__ == '__main__':
    rclpy.init()
    calibration_node = CalibrationNode()
    calibration_node.run_calibration()
    rclpy.spin(calibration_node)
    rclpy.shutdown()
