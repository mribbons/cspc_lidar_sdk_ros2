# Requirements
ROS2 Humble
COIN-D4 Lidar unit

# Setup

```sh
sudo apt update
sudo apt install ros-humble-pcl-ros pcl-tools
sudo apt install pcl-tools
mkdir myapp
cd myapp
git clone https://github.com/mribbons/cspc_lidar_sdk_ros2
source /opt/ros/humble/setup.bash
sudo cp cspc_lidar_sdk_ros2/sc_mini.rules /etc/udev/rules.d
colcon build --symlink-install
source install/setup.bash
ros2 launch cspc_lidar lidar_launch.py
ros2 launch cspc_lidar lidar_rviz.py
```
