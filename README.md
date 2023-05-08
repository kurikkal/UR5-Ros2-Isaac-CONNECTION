# UR5-Ros2-Isaac-CONNECTION
Documentation for the connection between ur5 &lt;-> ros2 &lt;-> Isaac Sim



## Install necessary packages

The following commands should install all necessary ROS2 Packages needed.

```
sudo apt install ros-humble-ur-*
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

```


## Step 2
Open the terminal and Run:
```
unset LD_LIBRARY_PATH
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export ROS_DOMAIN_ID=99
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
