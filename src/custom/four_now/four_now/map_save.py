import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class MapSave(Node):
    def __init__(self):
        super().__init__('point')
        self.map_saved = False
        self.overall_map_sub = None

        self.create_subscription(Bool, '/exploration_finish', self.finish_callback, 10)
        self.get_logger().info('mapSave Node Running...')

    def finish_callback(self, msg):
        if msg.data and not self.map_saved:
            self.get_logger().info('exploration finished, subscribing to /overall_map...')
            self.overall_map_sub = self.create_subscription(
                PointCloud2, '/overall_map', self.map_callback, 10
            )
            points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))



    def map_callback(self, msg):
        if self.map_saved:
            return

        # points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        filename = 'final_map.pcd'
        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for pt in points:
                f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")

        self.get_logger().info(f'saved map to {filename}')
        self.map_saved = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapSave())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
