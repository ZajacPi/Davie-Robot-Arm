import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard  # pip install keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'key_input', 10)
        self.get_logger().info("Keyboard node started. Use arrow keys (↑ ↓ ← →). Press ESC to quit.")
        self.create_timer(0.05, self.check_keys)

    def check_keys(self):
        if keyboard.is_pressed('up'):
            self.publish_key("up")
        elif keyboard.is_pressed('down'):
            self.publish_key("down")
        elif keyboard.is_pressed('left'):
            self.publish_key("left")
        elif keyboard.is_pressed('right'):
            self.publish_key("right")
        elif keyboard.is_pressed('esc'):
            self.get_logger().info("ESC pressed. Shutting down...")
            rclpy.shutdown()

    def publish_key(self, key: str):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {key}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
