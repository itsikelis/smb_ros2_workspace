import rclpy
from rclpy.node import Node
from open3d_slam_msgs.srv import SaveMap


class SaveMapNode(Node):
    def __init__(self):
        super().__init__('save_map_node')

        # Create service client
        self.cli = self.create_client(SaveMap, '/open3d/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SaveMap service...')

        # Prepare empty request
        self.req = SaveMap.Request()

        # Call service every 3 seconds
        self.timer = self.create_timer(3.0, self.send_request)

    def send_request(self):
        self.get_logger().info("Calling /open3d/save_map...")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
