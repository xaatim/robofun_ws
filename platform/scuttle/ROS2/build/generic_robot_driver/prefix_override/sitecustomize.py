import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hatim/Documents/Github/robofun_ws/platform/scuttle/ROS2/install/generic_robot_driver'
