import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        self.get_logger().info("Keyboard Controller Ready (w/s/a/d/q/x)")

        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent: {key}")
            if key == 'q':
                break

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
