import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uav/cs2_ws/install/ros_gz_crazyflie_control'
