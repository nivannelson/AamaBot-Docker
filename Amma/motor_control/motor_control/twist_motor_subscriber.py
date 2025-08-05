import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .motor_driver import MotorController

class TwistMotorSubscriber(Node):
    def __init__(self):
        super().__init__('twist_motor_subscriber')
        self.motor = MotorController()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info("Subscribed to /cmd_vel")

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        speed = int(linear * 255)  # full PWM range
        turn = int(angular * 255)

        self.get_logger().info(f"Received: linear={linear:.2f}, angular={angular:.2f}")

        if speed > 0:
            if turn > 0:
                self.motor.smooth_left(speed)
            elif turn < 0:
                self.motor.smooth_right(speed)
            else:
                self.motor.move_forward(speed)
        elif speed < 0:
            if turn > 0:
                self.motor.smooth_left(-speed)
            elif turn < 0:
                self.motor.smooth_right(-speed)
            else:
                self.motor.move_backward(-speed)
        elif turn > 0:
            self.motor.spin_counterclockwise(abs(turn))
        elif turn < 0:
            self.motor.spin_clockwise(abs(turn))
        else:
            self.motor.stop()

def main(args=None):
    rclpy.init(args=args)
    node = TwistMotorSubscriber()
    rclpy.spin(node)
    node.motor.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
