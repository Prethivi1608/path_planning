import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prethiviraj/ros2/workspaces/path_planning/install/pathplanning'
