import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aryan/Documents/GitHub/HowYouSeeMe/src/ros2_ws/install/howyouseeme_ros2'
