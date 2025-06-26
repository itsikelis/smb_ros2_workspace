import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess
import os

DURATION_IN_MINUTES: float = 10
TOPIC: str = "/way_point"
# TOPIC: str = "/goal_point"


class TimerKillNode(Node):
    def __init__(self):
        super().__init__("timer_kill_node")
        self.publisher = self.create_publisher(PointStamped, TOPIC, 10)
        self.get_logger().info(f"Starting {DURATION_IN_MINUTES}-minute timer...")
        self.timer = self.create_timer(DURATION_IN_MINUTES * 60, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(
            f"{DURATION_IN_MINUTES} minutes timeout passed. Sending shutdown transition to /tare_planner..."
        )

        # Use subprocess to run lifecycle command
        try:
            os.system("pkill -f tare_planner_node")
            self.get_logger().info("Successfully sent shutdown to /tare_planner_node")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to shut down /tare_planner: {e}")

        # Publish goal point
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = 0.0
        msg.point.y = 0.0
        msg.point.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Published to /goal_point. Shutting down...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TimerKillNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
