import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import evdev
    
class ControllerEvdevNode(Node):
    def __init__(self):
        super().__init__('controller_test_node')
        self.publisher_ = self.create_publisher(String, 'controller_events', 10)
        
        device_path = 'dev/input/event7'
        try:
            self.device = evdev.InputDevice(device_path)
            self.get_logger().info(f"Listening for inputs on {device_path}...")
        except FileNotFoundError:
                self.get_logger().info(f"Device {device_path} not found")
                self.device = None
                devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
                for device in devices:
                    print(device.path, device.name, device.phys)     
        if self.device:
            self.create_timer(0.01, self.read_events)

    def read_events(self):
        for event in self.device.read():
            if event.type == evdev.ecodes.EV_KEY:
                button= evdev.categorize(event)
                state = "pressed" if button.keystate else "released"
                msg = String()
                msg.data = f"Button {button.keycode} {state}"
                self.publisher_.publish(msg)
                self.get_logger.info("Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerEvdevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()