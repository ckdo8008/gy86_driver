import time
import rclpy
from rclpy.node import Node
import HMC5883L

class CompassNode(Node):
    def __init__(self):
        super().__init__('compass_node')
        # Initialize your HMC5883L class and other setups here
        self.compass = HMC5883L(gauss=4.7, declination=(5, 50))
        # You might want to set up a timer or subscriber here

    def run(self):
        while rclpy.ok():
            print(self.compass)
            time.sleep(0.05)

if __name__ == "__main__":
    rclpy.init()
    compass_node = CompassNode()
    try:
        compass_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        compass_node.destroy_node()
        rclpy.shutdown()
