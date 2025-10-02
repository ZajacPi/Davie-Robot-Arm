import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')
        self.subscription = self.create_subscription(
            String,
            'controller_events',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received event: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()