import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from custom_msgs.srv import Robot2Map

class TransformServiceNode(Node):
    def __init__(self):
        super().__init__('robot_to_map_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(Robot2Map, 'transform_point_robot2map', self.handle_request)
        self.get_logger().info('transform_point_robot2map service ready.')

    def handle_request(self, request, response):
        try:
            point_in_map_frame = self.tf_buffer.transform(
                request.point_in_robot_frame,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            response.transformed_point = point_in_map_frame
            self.get_logger().info(f"Transformed point: {point_in_map_frame.point}")
        except Exception as e:
            self.get_logger().error(f"Transform failed: {str(e)}")
            response.transformed_point = PointStamped()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TransformServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
