import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .motor_driver import MotorController


class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'motor_cmd',
            self.listener_callback,
            10)
        self.motor = MotorController()
        self.get_logger().info("Motor Subscriber Started")

    def listener_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'w':
            self.motor.move_forward(50)
        elif cmd == 's':
            self.motor.move_backward(50)
        elif cmd == 'a':
            self.motor.turn_left(50)
        elif cmd == 'd':
            self.motor.turn_right(50)
        elif cmd == 'z':
            self.motor.spin_counterclockwise(50)
        elif cmd == 'c':
            self.motor.spin_clockwise(50)
        elif cmd == 'x':
            self.motor.stop()
        else:
            self.get_logger().info("Unknown command")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.motor.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
