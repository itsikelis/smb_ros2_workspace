import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from object_detection_msgs.msg._object_detection_info import ObjectDetectionInfo
from object_detection_msgs.msg._object_detection_info_array import (
    ObjectDetectionInfoArray,
)
import tf2_ros
import tf2_geometry_msgs
import pandas as pd
from typing import Optional

THRESHOLD_FOR_PROBABILITY_TO_BE_DETECTION = 0.5


class TransformServiceNode(Node):
    def __init__(self):
        super().__init__("robot_to_map_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.srv = self.create_service(Robot2Map, 'transform_point_robot2map', self.handle_request)
        self.subscription = self.create_subscription(
            ObjectDetectionInfoArray, "/detection_info", self.listener_callback, 10
        )

        # TODO: define the topic
        self.subscription = self.create_subscription(
            Empty, "/write_to_file", self.write_to_file_callback, 10
        )

        self.collected_obs = pd.DataFrame(columns=["class", "x", "y", "z"])
        self.get_logger().info("transform_point_robot2map service ready.")

    def get_transformed_point(
        self, point: Point, original_frame: str, destination_frame: str
    ) -> Optional[PointStamped]:
        point_in = PointStamped()
        point_in.header.frame_id = (
            original_frame  # TODO: Update to match your actual robot frame
        )
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

    def add_point_to_df(self, point_to_add: PointStamped, class_name: str):
        new_point = {
            "class": class_name,
            "x": point_to_add.point.x,
            "y": point_to_add.point.y,
            "z": point_to_add.point.z,
        }
        self.collected_obs = pd.concat(
            [self.collected_obs, pd.DataFrame([new_point])], ignore_index=True
        )

    def listener_callback(self, msgs: ObjectDetectionInfoArray):
        assert len(msgs.info) > 0, "No detections received."
        msg = msgs.info[0]
        if msg.confidence < THRESHOLD_FOR_PROBABILITY_TO_BE_DETECTION:
            return
        # TODO: add correct frame
        point_in_map_frame = self.get_transformed_point(
            msg.position, "base_link", "map"
        )
        assert point_in_map_frame is not None
        assert isinstance(point_in_map_frame, PointStamped)

        self.add_point_to_df(point_in_map_frame, msg.class_id)
        self.get_logger().info(f"Entire df: {self.collected_obs}")

    def write_to_file_callback(self, msg: Empty):
        file_path = "points_transformed.csv"
        self.collected_obs.to_csv(file_path, index=False)
        self.get_logger().info(
            f"Saved {len(self.collected_obs)} points to '{file_path}'."
        )


def main(args=None):
    rclpy.init(args=args)
    node = TransformServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
