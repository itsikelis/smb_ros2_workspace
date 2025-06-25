import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        #create publisher with queue=10
        self.publisher_ = self.create_publisher(PointStamped, '/way_point', 10)
        #publish every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

        #FSM WAITING -> ENTER -> EXPLORE -> EXIT
        self.state = 'WAITING'
        self.index = 0
        self.current_waypoints = None
        #

    def timer_callback(self):
        if self.state == 'WAITING':
            self.get_logger().info('Starting FSM... ENTER, EXPLORE, EXIT')
            self.state = 'ENTER_BUILDING'
        elif self.state == 'ENTER_BUILDING':
            self.get_logger().info('Entering building...')
            self.current_waypoints = self.waypoints_enter
            msg = PointStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'  # Common frame for waypoints
            msg.point.x = 5
            msg.point.y = 0
            msg.point.z = 0
            self.publisher_.publish(msg)
        elif self.state == 'EXPLORE':
            self.get_logger().info('Exploring...')
            # Here you would implement the exploration logic
            # For now, we just simulate it by moving to the next waypoint
        if self.state == 'EXIT_BUILDING':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()