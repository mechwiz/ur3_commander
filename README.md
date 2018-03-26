# UR3 Commander
**Michael Wiznitzer**

Coding Challenge


## Introduction
####  Objective
The goal of this project is to control a UR3 robot with webcam input.

#### How it works
- The UR3 is simulated in a Gazebo environment provided by the [Universal Robot](https://github.com/ros-industrial/universal_robot) package.
- An AR tag is used as the webcam input to provide end-effector goal positions in 3D space. The [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package was used for this.
- MoveIt is used for motion planning - specifically, the python moveit-commander module.
- An Rviz interface also provided by the [Universal Robot](https://github.com/ros-industrial/universal_robot) package is used for vision feedback and for displaying planned trajectories.


## Installation
#### Prerequisits
- Linux running on Ubuntu 16.04 with ROS Kinetic
- Calibrated webcam. Follow this [tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) to accomplish this using a checkerboard. Make sure you enter in the right port for your video camera in the usb_cam launch file [here](launch/usb_cam-test.launch).
- An AR Tag. Print out this [sheet](imgs/artags) and cut out the top left tag.

#### Dependencies
To install this package, first open up a terminal and install moveit.
```bash
sudo apt-get install ros-kinetic-moveit
```

Then perform the following steps in your home directory:
```bash
mkdir -p ur3_ws/src; cd ur3_ws/src; catkin_init_workspace
git clone git@github.com:mechwiz/ur3_commander.git
cd ur3_ws; rosdep install --from-paths . --ignore-src --rosdistro=kinetic
cd ur3_ws; catkin_make; source devel/setup.bash
```

All of the required dependencies should be included (Gazebo, Rviz, ar_track_alvar, usb_cam, moveit, etc...). This should hopefully just be a plug and play package.

## Implementation
#### Running The Simulation
Note, wait until each of the following launch files have finished starting before running the next one.

- Launch the Gazebo world: `roslaunch ur3_commander ur3.launch limited:=true`
- Launch the Moveit Setup configs to allow for motion planning: `roslaunch ur3_commander ur3_moveit_planning_execution.launch sim:=true`
- Launch Rviz with the motion planning plugin: `roslaunch ur3_commander moveit_rviz.launch config:=true`
    - Note, that the [UR3KinematicsPlugin](config/kinematics.yaml) is used instead of the KDLKinematicsPlugin as it seems to do a better job
- Launch the computer vision node and setup configs as well as the moveit_commander node `roslaunch ur3_commander artag_startup.launch`
    - Note, that this file also launches a static tf between the camera frame `usb_cam` and the gazebo `world` frame so that the robot and camera can be in the same frame.


#### Nodes
##### Computer Vision Node
[`getArPos.py`](src/getArPos.py)

This node takes the x, y, and z positions in meters (relative to your camera frame) of the AR tag returned by the `/ar_pose_marker` topic and outputs them in cm on a figure window like the one shown in the demo below so that the user can get some position feedback of where the tag is relative to the robot frame. Note that the 'z' axis is mapped to distance from the camera whereas x and y is in a plane parallel to your laptop screen. The screen shows different messages depending on the stage the program is in. This is outlined below:
- Once everything is fired up, the message "Calibrate 0 for Z. Hit 'c' when done." is shown. This allows the user to adjust at what distance from the camera frame should be considered zero. Additionally, the user can recalibrate at any time during the session. This is important because the closer the tag is to the screen, the smaller amount of room there is to move on the x and y axes due to the fact that the tag is taking up a significant amount of the screen. However, if the 'zero' position can be adjusted to be at a further distance where the AR tag is samller in the screen, then there is a lot more room to move on the x and y axes.
- After the inital calibration, the message "Move Tag around to select a postion. Hit 's' when done." appears. This tells the user to move around the tag (which appears as a blue sphere marker in Rviz) and hit the 's' key when he wants to send a position to the moveit_commander node for motion planning. Once, this occurs, a message saying "Planning..." will appear on the screen, though it might disapear quickly depending on how long the planning takes.
- Once planning is done, a message saying "Dispalying Plan & Executing. Wait 10 sec..." appears on the screen. The user can watch a planned trajectory being displayed in Rviz followed by the exectution of it which can be seen in both Gazebo and Rviz. If a plan is not found, then a message saying "No Plan Found. Try another point" is displayed to the user to pick another point.

Subscribed Topics:
- `/usb_cam/image_raw` - Webcam image data
- `/ar_pose_marker` - AR tag position in 3D space relative to the camera frame
- `/visualization_marker` - The AR package sends a marker corresponding to the AR tag over to Rviz. This topic is being subscribed to in order to publish the marker on a different topic with the corrected z-axis calibration and to be shown as sphere
- `robot_state` - The moveit_commander node sends over the status of what state the move_group is in - planning, displaying & executing, no plan found, or ready to recieve new position. Depending on the state, different messages are shown on the window display as discussed above

Published Topic:
- `ar_point` - This is the topic that new positions published as Point messages are sent to the moveit_commander node
- `visualization_marker_update` - This is the updated marker (that takes into account the calibrated zero postion) sent over to Rviz


##### MoveIt Commander Node
[`move2pos.py`](src/gridcalc.py)

This node implements the python moveit_commander module to interact with the moveit motion-planning plugin in Rviz. It is controlling the "manipulator" move-group of the UR3 robot and is using the following configs:
- The RRTConnectkConfigDefault Planner which seems to be a suggest planner to use based off browsing the web
- A planning time of 5 seconds to give the planner ample time to find solutions
- The ability to attempt to find a plan 5 times if it fails at first
- A goal position tolerance of 0.05 meters mainly because we don't need to be so restrictive for this demo

Once a point is recieved over the `ar_point` topic, the node implements the following steps
 1. Set the new pose message with the new point
 2. Publish over the `robot_state` topic that the node is in the planning stage
 2. Clear any previously target Pose positions
 3. Set the new pose target with the freshly made pose message
 4. Set the start state of the robot to the current state to make sure that the planning start state will have the most latest current position.
 5. Plan a trajectory
 6. If a plan has been found, publish over the `robot_state` topic that the node is in the "show trajectory and execute" state. A trajectory will proceed to be shown in Rviz and then will be executed. Reset the robot_state to "ready-for-next-position" after 10 seconds
 7. If no plan has been found, publish over the `robot_state` topic that the node has not found a plan. Reset the robot_state to "ready-for-next-position" after 2 seconds

Subscribed Topics:
- `/move_group/feedback` for deteriming the current state of the robot
- `ar_point` - For recieving the Point messages sent over by the computer vision node

Published Topic:
- `robot_state` - For sending over the current robot state to the computer vision node

## Demo & Troubleshooting
#### Video
A video of this package in action is shown below: ![here](imgs/ur3_commander_demo.gif).

#### Troubleshooting
- It may happen that the robot ends up in some convoluted position where no plan can be achieved. If this happens, your best bet is to relaunch the package.
- An attempt was made to include the [ur_modern_driver](https://github.com/ThomasTimm/ur_modern_driver) by Thomas Timm for better functionality than the current ur_driver in the Universal_Robot package. However, based on reasearch done, it seems like this driver only works when connected to a real robot.
