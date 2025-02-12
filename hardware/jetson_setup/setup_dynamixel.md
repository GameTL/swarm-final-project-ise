sudo apt-get install ros-humble-dynamixel-sdk 
or this
better way 

mkdir -p ~/robotis_wd/src
cd robotis_wd/src/
git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK
colcon build --symlink-install
cd robotis_wd
. install/local_setup.bash
sudo chmod 777 /dev/ttyUSB0


ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 0}"