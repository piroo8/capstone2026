import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetson/rob498_2026/capstone2026/ros2_ws/install/my_package'
