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

## Install & Setup URCap on teach pendant
For using the ur_robot_driver with a real robot you need to install the externalcontrol-1.0.5.urcap. Download it from https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases and move it to an USB stick.

On the welcome screen select Setup Robot and then URCaps to enter the URCaps installation screen.
There, click the little plus sign at the bottom to open the file selector. There you should see all urcap files stored inside the robot’s programs folder or a plugged USB drive. Select and open the externalcontrol-1.0.5.urcap file and click open. Your URCaps view should now show the External Control in the list of active URCaps and a notification to restart the robot. Do that now.
After reboot, select Program Robot on the welcome screen, select the Installation tab and 'External Control' option will be there.
Setup the IP address of the external PC which will be running the ROS driver. Note that the robot and the external PC have to be in the same network, ideally in a direct connection with each other to minimize network disturbances. The custom port should be left untouched.

```
Remote host IP : 192.168.1.101
```
To use the new URCaps, create a new program and insert the External Control program node into the program tree,save the program.

## Network Setup
Connect the UR control box directly to the remote PC with an ethernet cable.
Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:
```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```
On the remote PC, turn off all network devices except the "wired connection", e.g. turn off wifi.
Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```



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
