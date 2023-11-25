import rclpy
from rclpy.node import Node
import MPU6050

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.mpu = MPU6050(0x68)

    def run(self):
        while rclpy.ok():
            print("Temperature: ", self.mpu.get_temp())
            accel_data = self.mpu.get_accel_data()
            print("Accelerometer: ", accel_data)
            gyro_data = self.mpu.get_gyro_data()
            print("Gyroscope: ", gyro_data)
            rclpy.spin_once(self, timeout_sec=1)  # Adjust the timeout as needed

if __name__ == "__main__":
    rclpy.init()
    mpu_node = MPU6050Node()
    try:
        mpu_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        mpu_node.destroy_node()
        rclpy.shutdown()
