# Arm package for ROS

Repository for the ros package of the robotic arm

## Dependencies

### [ROS](http://wiki.ros.org/ROS/Installation)

```shell
export ROS_VERSION=kinetic
```
or
```shell
export ROS_VERSION=melodic
```

### ROS Serial for the Microcontroler

[Here the repository with the microcontroler code and instructions to start the rosserial](https://gitlab.com/pequihome/arm-micro)


### Move It
```shell
sudo apt-get install ros-$ROS_VERSION-catkin python-catkin-tools
sudo apt install ros-$ROS_VERSION-moveit
```
Create a catkin workspace:
```shell
mkdir -p ~/roboga_ws/src
cd ~/roboga_ws/src
git clone -b $ROS_VERSION-devel https://github.com/ros-planning/moveit_tutorials.git
git clone -b $ROS_VERSION-devel https://github.com/ros-planning/panda_moveit_config.git
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_VERSION
cd ..
catkin config
catkin build
```

### Python dependencies
```shell
pip install rospkg
pip install rospy
pip install pyyaml
pip install pyqt5
pip install pyserial
```

## This package 'roboga_arm'
```shell
cd ~/roboga_ws/src
git clone https://gitlab.com/pequihome/roboga_arm
cd ..
catkin build
```


## Running package

```shell
source ~/roboga_ws/devel/setup.bash
```

### Base
The launch will open rviz with the model of the arm, and the topic needed for the format of message recived by the microcontroller
```shell
roslaunch roboga_arm roboga.launch
```
If the model apears collapsed on ROS Melodic, try this:
```shell
export LC_NUMERIC="en_US.UTF-8"
```

### Ros serial
In another terminal open the rosserial node to comunicate with the microcontroler   
Example command if serial is connected on **/dev/ttyUSB0**
```shell
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=250000
```

### Example pick and place
In another terminal :
```shell
rosrun roboga_arm roboga_grasp.py
```