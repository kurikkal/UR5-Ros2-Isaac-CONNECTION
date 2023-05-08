# UR5-Ros2-Isaac-CONNECTION
Documentation for the connection between ur5 &lt;-> ros2 &lt;-> Isaac Sim

# UR5-ROS2 Connection

## Install necessary packages

The following commands should install all necessary ROS2 Packages needed.

```
sudo apt-get install ros-humble-ur-robot-driver
sudo apt install ros-humble-ur-*
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

```

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




## Install & Setup URCap on teach pendant
For using the ur_robot_driver with a real robot you need to install the externalcontrol-1.0.5.urcap. Download it from https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases and move it to an USB stick.

On the welcome screen select Setup Robot and then URCaps to enter the URCaps installation screen.
There, click the little plus sign at the bottom to open the file selector. There you should see all urcap files stored inside the robot’s programs folder or a plugged USB drive. Select and open the externalcontrol-1.0.5.urcap file and click open. Your URCaps view should now show the External Control in the list of active URCaps and a notification to restart the robot. Do that now.
After reboot, select Program Robot on the welcome screen, select the Installation tab and 'External Control' option will be there.
Setup the IP address of the external PC which will be running the ROS driver. Note that the robot and the external PC have to be in the same network, ideally in a direct connection with each other to minimize network disturbances. The custom port should be left untouched.

```
Remote host IP : 192.168.1.101
```

## Program the UR5
Select 'Program Robot', create a new program and insert the External Control program node into the program tree,save the program.
If enabled by default, disable the ethernet in installation tab.
The UR5 is ready for connection.


## Extract calibration information
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.
Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

Verify the connection from the PC with e.g. ping.
```
ping 192.168.1.102
```
Run the following commands.

```
source /opt/ros/humble/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"

```

## Usage
Run the command in ROS2 sourced terminal:

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=true
```

RViz will be launched with the robot depicting its current state.

Press the play button in teach pendant to enable it to receive commands

Try it with 'move it' to plan and execute path

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

# ROS2- Isaac Sim Connection

## Step 1
Move the 'fastdds.xml' file also located at the root of the ros2_workspace folder in Isaac Sim folder to '~/.ros/'

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

## Step 4
Open action graph. Search for ROS2 Context and drag it into the empty graph. Type the same domain ID you choose in the node. Edit the rest of the action graph according to the project.

## Step 5
 Go to the terminal in Step 3, Source ROS2 there:
 ```
 source /opt/ros/humble/setup.bash
 
 ```
 Now the bridge connection is established.

# Data from UR5 to Isaac

## Action Graph

To subscribe to joint_states of the real UR5 and articulate the simulated version in Isaac Sim. Follow the below steps.

Launch Isaac Sim following the steps in above section.
Load the UR5.usd
Open action graph
Add the nodes as shown in the image to the graph and connect accordingly.
![image](https://user-images.githubusercontent.com/72142589/236913790-c8f173ed-1194-40ae-bf2e-2947d536bbe4.png)

Enter Domain Id:99 in Ros2 Context node

Enter Target: /World/UR5 and uncheck UsePath in Articulation Controller.

Enter topic : joint_states

## Launch the UR Control
In the terminal in the above mentioned section (ROS2- Isaac Sim Connection/Step 2)
Source ROS2
Run:

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=true
```

Rviz window will pop up.

Open another terminal, run all the commands mentioned in 'Step 2' and source ROS2.
Run:
```
ros2 topic list
```

A set of ROS2 topics will be shown. Ensure the topic  'joint_states' is running

## Start Simulation
Press the play button in Isaac Sim. The UR5 in isaac sim will move to the initial position of the real robot.

Go to the the 'Move' tab in teach pendant and move the robot. The robot in Isaac Sim should move accordingly.

