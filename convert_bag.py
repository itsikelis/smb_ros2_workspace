# import sys
# import rosbag2_py
# import open3d as o3d

# from pathlib import Path
# from rosidl_runtime_py.utilities import get_message
# from rclpy.serialization import deserialize_message

# from sensor_msgs_py.point_cloud2 import read_points

# # ros2 bag record /overall_map /tf /tf_static -o exploration_bag
# # python3 convert_bag.py exploration_bag /overall_map

# def get_rosbag_options(path, serialization_format='cdr'):
#     storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

#     converter_options = rosbag2_py.ConverterOptions(
#         input_serialization_format=serialization_format,
#         output_serialization_format=serialization_format)

#     return storage_options, converter_options


# if len(sys.argv) < 3:
#     print("Usage: python3 bag_to_pcd.py <bagfile> <pc_topic>")
#     sys.exit(1)

# bag_path = Path(sys.argv[1])
# pcd_dir = bag_path.parent / "pcd"
# if not pcd_dir.exists():
#     pcd_dir.mkdir()
# print(f"Converting {bag_path.absolute()} to pcd files...")
# pc_topic = sys.argv[2]

# reader = rosbag2_py.SequentialReader()
# storage_options, converter_options = get_rosbag_options(str(bag_path))
# reader.open(storage_options, converter_options)

# topic_types = reader.get_all_topics_and_types()
# type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
# fields = ('x', 'y', 'z', 'intensity', 'ring', 'time')

# total_pcd = o3d.geometry.PointCloud()
# while reader.has_next():
#     (topic, data, t) = reader.read_next()
#     if topic != pc_topic:
#         continue
#     points = []
    
#     msg_type = get_message(type_map[topic])
#     msg = deserialize_message(data, msg_type)
#     ros_points = read_points(msg, field_names=fields, skip_nans=True)
#     for p in ros_points:
#         points.append([p[0], p[1], p[2]])
    
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     o3d.io.write_point_cloud(f"{pcd_dir}/{t}.pcd", pcd)
#     total_pcd += pcd
#     print(f"Saved {pcd_dir}/{t}.pcd")

# o3d.io.write_point_cloud(f"total.pcd", total_pcd)