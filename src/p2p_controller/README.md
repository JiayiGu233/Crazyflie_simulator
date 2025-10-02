source /opt/ros/humble/setup.bash
colcon build --symlink-install 
source install/setup.bash
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py


another terminal:
source install/setup.bash
ros2 run p2p_controller move