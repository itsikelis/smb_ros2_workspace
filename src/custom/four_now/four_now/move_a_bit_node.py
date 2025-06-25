import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time

class ConstantTwistPublisher(Node):

    def __init__(self):
        super().__init__('move_a_bit')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Timer to publish every 3 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)

        # Define constant twist message
        self.constant_twist = TwistStamped()
        self.constant_twist.twist.linear.x = 0.0
        self.constant_twist.twist.linear.y = 0.0
        self.constant_twist.twist.linear.z = 0.0
        self.constant_twist.twist.angular.x = 0.0
        self.constant_twist.twist.angular.y = 0.0
        self.constant_twist.twist.angular.z = 1.0

        self.get_logger().info("Constant twist publisher initialized.")

    def publish_twist(self):
        self.constant_twist.header.stamp = self.get_clock().now().to_msg()
        self.constant_twist.header.frame_id = "base_link"  # or use whatever your robot expects
        self.publisher_.publish(self.constant_twist)
        self.get_logger().info(f'Published twist: {self.constant_twist.twist.angular.z} rad/s around Z')

def main(args=None):
    rclpy.init(args=args)
    node = ConstantTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
