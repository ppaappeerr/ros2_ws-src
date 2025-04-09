import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/p/ros2_ws/src/scan_to_pointcloud/install/scan_to_pointcloud'
