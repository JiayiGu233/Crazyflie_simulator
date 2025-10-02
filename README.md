```bash
cd ~/cs2_ws
sudo apt-get update

rosdep update
rosdep install --from-paths src -y --ignore-src

cd ~/cs2_ws/src/crazyswarm2
git submodule update --init --recursive

cd ~/cs2_ws
colcon build --symlink-install

sudo apt-get update
sudo apt-get update --fix-missing
index expired:
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo apt install -y ros-humble-ros-gz-sim
sudo apt-get install -y ros-humble-ros-gz-bridge

uav@CLL-LT05:~/cs2_ws$ echo 'export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/cs2_ws/src/crazyflie-simulation/simulator_files/gazebo' >> ~/.bashrc
uav@CLL-LT05:~/cs2_ws$ echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/cs2_ws/src/crazyflie-simulation/simulator_files/gazebo' >> ~/.bashrc
uav@CLL-LT05:~/cs2_ws$ source ~/.bashrc

uav@CLL-LT05:~/cs2_ws$ source install/setup.bash 
uav@CLL-LT05:~/cs2_ws$ ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py
