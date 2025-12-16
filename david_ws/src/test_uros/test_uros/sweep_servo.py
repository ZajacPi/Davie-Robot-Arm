import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class ServoTest(Node):
    def __init__(self):
        super().__init__("sweep_servo")
        self.pub = self.create_publisher(Float32, "/servo_angle", 10)

        while rclpy.ok():
            for a in range(0, 181, 1):
                self.pub.publish(Float32(data=float(a)))
                time.sleep(0.03)
            for a in range(180, -1, -1):
                self.pub.publish(Float32(data=float(a)))
                time.sleep(0.03)

rclpy.init()
ServoTest()
rclpy.shutdown()
