import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prethivi/ros2_ws/pathplanning/install/pathplanning'
