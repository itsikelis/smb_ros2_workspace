import rclpy
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math

PERIOD = 0.01  # timer period (s)
SCALE = 5.0  # rad/s for cmd_vel.angular.z
WINDOW_SIZE = 5  # number of Odometry samples kept (~0.5 s)
STOP_DIST_THRESH = 0.0001  # planar displacement (m) to treat as “stopped”
TWIST_DURATION = 20.0  # seconds before flipping direction


class ConstantTwistPublisher(Node):

    def __init__(self):
        super().__init__("move_a_bit_node")

        # Publisher & subscriber
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.state_sub = self.create_subscription(
            Odometry, "/state_estimation", self.se_callback, 10
        )

        # History used for “stopped” detection
        self.pos_history = deque(maxlen=WINDOW_SIZE)
        self.is_stopped = False

        # Twisting-state bookkeeping
        self.sign = -1.0
        self.twist_active = False
        self.twist_start_time = None  # rclpy.time.Time when twisting began

        # Main timer
        self.timer = self.create_timer(PERIOD, self.publish_twist)

        self.get_logger().info("Constant-twist controller ready.")

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------
    def se_callback(self, msg: Odometry):
        # Track absolute planar position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pos_history.append((x, y))

        if len(self.pos_history) < WINDOW_SIZE:
            self.is_stopped = False
            return

        # Max displacement over the recent window
        xs, ys = zip(*self.pos_history)
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dist = math.hypot(dx, dy)

        self.is_stopped = dist < STOP_DIST_THRESH
        if self.is_stopped:
            self.get_logger().info("Robot is stopped.")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def create_twist_command(self) -> TwistStamped:
        twist = TwistStamped()
        twist.twist.angular.z = self.sign * SCALE
        return twist

    def switch_sign(self):
        self.sign = 1.0 if self.sign < 0.0 else -1.0

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def publish_twist(self):
        now = self.get_clock().now()

        if not self.is_stopped:
            # Robot moved → stop twisting immediately & reset counter
            self.twist_active = False
            return

        # Robot is stopped: start or continue twisting
        if not self.twist_active:
            self.switch_sign()
            # First frame of a “stopped” interval
            self.twist_active = True
            self.twist_start_time = now

        # Publish the twist
        cmd = self.create_twist_command()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = "base_link"
        self.cmd_pub.publish(cmd)

        # After TWIST_DURATION seconds, flip direction and restart timer
        elapsed = (now - self.twist_start_time).nanoseconds / 1e9
        if elapsed >= TWIST_DURATION:
            self.switch_sign()
            self.twist_start_time = now
            self.get_logger().info(
                f"{TWIST_DURATION:.0f}s elapsed - switching direction to {self.sign * SCALE} rad/s"
            )


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
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


if __name__ == "__main__":
    main()
