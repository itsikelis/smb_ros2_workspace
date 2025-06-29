import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped, Point
from object_detection_msgs.msg import ObjectDetectionInfo, ObjectDetectionInfoArray
import tf2_ros
import tf2_geometry_msgs
from typing import Optional

THRESHOLD_FOR_PROBABILITY_TO_BE_DETECTION = 0.4


class TransformServiceNode(Node):
    def __init__(self):
        super().__init__("robot_to_map_node")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.subscription = self.create_subscription(
            ObjectDetectionInfoArray, "/detection_info", self.listener_callback, 10
        )
        self.sub_to_write_from_user = self.create_subscription(
            Empty, "/write_detections_to_file", self.write_to_file_callback, 10
        )
        self.sub_to_write_from_tare = self.create_subscription(
            Empty, "/exploration_finish", self.write_to_file_callback, 10
        )

        # Data collection
        self.collected_obs = (
            []
        )  # List of dicts: [{"class": ..., "x": ..., "y": ..., "z": ...}]
        self.counter = 0
        self.skip_rate = 1  # Process every nth detection

        self.get_logger().info("TransformServiceNode initialized and ready.")

    def get_transformed_point(
        self, point: Point, original_frame: str, destination_frame: str
    ) -> Optional[PointStamped]:
        point_in = PointStamped()
        point_in.header.frame_id = original_frame
        point_in.header.stamp = self.get_clock().now().to_msg()
        point_in.point = point

        try:
            point_transformed = self.tf_buffer.transform(
                point_in,
                destination_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            self.get_logger().info(f"Transformed point: {point_transformed}")
            return point_transformed
        except Exception as e:
            self.get_logger().error(f"Transform failed: {str(e)}")
            return None

    def add_point_to_df(
        self, point_to_add: PointStamped, class_name: str, confidence: float
    ):
        new_point = {
            "class": class_name,
            "x": point_to_add.point.x,
            "y": point_to_add.point.y,
            "z": point_to_add.point.z,
            "confidence": confidence,
        }
        self.collected_obs.append(new_point)

    def update_df(self, msg: ObjectDetectionInfo):
        transformed = self.get_transformed_point(
            msg.position, original_frame="base_link", destination_frame="map"
        )
        if transformed:
            self.add_point_to_df(transformed, msg.class_id, msg.confidence)

    def listener_callback(self, msgs: ObjectDetectionInfoArray):
        if not msgs.info:
            self.get_logger().warn("Received empty detection array.")
            return

        to_add = 1
        for msg in msgs.info:
            if msg.confidence < THRESHOLD_FOR_PROBABILITY_TO_BE_DETECTION:
                continue

            self.counter += to_add
            to_add = 0

            if self.counter % self.skip_rate == 0:
                self.counter = 0
                self.get_logger().info(f'Received detection: "{msg.class_id}"')
                self.update_df(msg)

        # Display current state (as text)
        display_str = "\n".join(
            f'{obs["class"]}: x={obs["x"]:.2f}, y={obs["y"]:.2f}, z={obs["z"]:.2f}, confidence={obs["confidence"]:.2f}'
            for obs in self.collected_obs
        )
        self.get_logger().info(f"Current collected points:\n{display_str}")

    def write_to_file_callback(self, msg: Empty):
        file_path = "points_transformed.csv"
        try:
            with open(file_path, "w") as f:
                f.write("class,x,y,z,confidence\n")
                for obs in self.collected_obs:
                    f.write(
                        f'{obs["class"]},{obs["x"]},{obs["y"]},{obs["z"]},{obs["confidence"]}\n'
                    )
            self.get_logger().info(
                f"Saved {len(self.collected_obs)} points to '{file_path}'."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write to file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
