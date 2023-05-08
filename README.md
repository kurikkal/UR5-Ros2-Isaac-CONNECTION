# UR5-Ros2-Isaac-CONNECTION
Documentation for the connection between ur5 &lt;-> ros2 &lt;-> Isaac Sim



## Install necessary packages

The following commands should install all necessary ROS2 Packages needed.

```
sudo apt-get install ros-humble-ur-robot-driver
sudo apt install ros-humble-ur-*
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

```

## Install URCap on teach pendant
For using the ur_robot_driver with a real robot you need to install the externalcontrol-1.0.5.urcap. Download it from https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases and move it to an USB stick.

On the welcome screen select Setup Robot and then URCaps to enter the URCaps installation screen.


## Extract calibration information

```
source /opt/ros/humble/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"

```

Choose a domain ID(eg:99)

## Step 3
Launch Isaac Sim

IMP: Do not source ROS2 in the terminal running Isaac Sim. So do not add the command to source ROS2 and the command to set ROS2 Domain ID to ~/.bashrc file.

## Step 3
After Isaac Sim is launched. Enable ROS2 Bridge in Isaac Sim ' Windows/Extensions'. ROS Bridge is enabled on default. It should be disabled first.

## Step 4
Open action graph. Search for ROS2 Context and drag it into the empty graph. Type the same domain ID you choose in the node. Edit the rest of the action graph according to the project.

## Step 5
 Go to the terminal in Step 3, Source ROS2 there:
 ```
 source /opt/ros/humble/setup.bash
 
 ```
 
 Now the bridge connection is established.
