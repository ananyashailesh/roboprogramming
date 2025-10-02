TurtleBot ROS Line Follower with T-Junction Detection

A self-contained ROS Noetic + Gazebo project that simulates a simple differential-drive robot with a camera and a vision-based line follower. The robot follows a main line, detects a T-junction, performs a left turn, searches for the left branch, and continues following.

Repo contents (what you provided)
turtle_line_ws/
└── src/
    └── turtle_line_follower/
        ├── urdf/
        │   └── turtle_bot.urdf
        ├── worlds/
        │   └── line_track.world
        ├── launch/
        │   └── turtle_line_follower.launch
        ├── scripts/
        │   └── line_follower.py
        ├── package.xml
        └── CMakeLists.txt

Quick README (copy-paste friendly)
Overview

This package (turtle_line_follower) provides:

turtle_bot.urdf — simple robot URDF with:

libgazebo_ros_diff_drive.so plugin (subscribes to /cmd_vel)

libgazebo_ros_camera.so camera plugin publishing /turtle_camera/image_raw (topic name depends on plugin config)

line_track.world — Gazebo world with a T-junction line track

line_follower.py — Python node implementing a PD controller + state machine for T-junction detection and left-turn behavior

turtle_line_follower.launch — launches Gazebo, spawns the URDF and starts the node

Prerequisites (Ubuntu + ROS Noetic)

Install required system packages:

sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-plugins \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-robot-state-publisher \
  python3-opencv

Setup (create workspace & package)

Run these commands exactly (assumes fresh start, change paths if needed):

# create workspace
mkdir -p ~/turtle_line_ws/src
cd ~/turtle_line_ws
catkin_make
echo "source ~/turtle_line_ws/devel/setup.bash" >> ~/.bashrc
source ~/turtle_line_ws/devel/setup.bash

# create the package (if not already created)
cd ~/turtle_line_ws/src
catkin_create_pkg turtle_line_follower \
  rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport robot_state_publisher tf gazebo_ros

# copy the following folders/files into:
# ~/turtle_line_ws/src/turtle_line_follower/{urdf,worlds,launch,scripts,CMakeLists.txt,package.xml}


Make the script executable:

chmod +x ~/turtle_line_ws/src/turtle_line_follower/scripts/line_follower.py


Build and source:

cd ~/turtle_line_ws
catkin_make
source devel/setup.bash

Run the simulation

Launch the whole system:

roslaunch turtle_line_follower turtle_line_follower.launch


If you prefer to launch components manually:

Terminal A — Gazebo world:

roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find turtle_line_follower)/worlds/line_track.world gui:=true


Terminal B — Spawn the URDF (if not using launch):

rosparam set robot_description -t $(rospack find turtle_line_follower)/urdf/turtle_bot.urdf
rosrun gazebo_ros spawn_model -param robot_description -urdf -model turtle_bot -x -2.5 -y 0 -z 0.1


Terminal C — Run the node:

rosrun turtle_line_follower line_follower.py

Expected behavior

Gazebo opens with the line_track.world (T-junction).

Robot turtle_bot spawns near the track.

The Python node prints log messages:

Line Follower Initialized. State: FOLLOWING

Following line - Error: ...

JUNCTION DETECTED! Initiating left turn sequence

Turn completed. Searching for left branch line...

Found left branch line!

OpenCV window (Line Detection) shows ROI and performance overlays.

Robot follows the main line, detects the T-junction, turns left, searches and follows the left branch.

Key topics & nodes to verify

Check these after launch:

rosnode list
rostopic list | egrep -i 'image|camera|cmd_vel|odom|tf'
rostopic info /cmd_vel


line_follower node subscribes to camera topic (in your script it subscribes to /turtle_camera/image_raw).

/cmd_vel must have a subscriber (diff_drive gazebo plugin) for the robot to move.

If the camera topic name differs (check rostopic list), either:

update the topic in scripts/line_follower.py:

self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)


or remap in launch/turtle_line_follower.launch:

<node name="line_follower" pkg="turtle_line_follower" type="line_follower.py" output="screen">
  <remap from="/turtle_camera/image_raw" to="/camera/image_raw"/>
</node>

Troubleshooting (quick list)
Robot not moving

Run: rostopic info /cmd_vel — if Subscribers: is empty, the diff_drive plugin didn't load.

Inspect Gazebo/roslaunch console for plugin load errors (missing libgazebo_ros_diff_drive.so).

Ensure gazebo_ros & gazebo_plugins are installed.

No camera frames

rostopic list | egrep -i 'image|camera' — check the camera topic name.

If no camera topic, ensure libgazebo_ros_camera.so is available (install ros-noetic-gazebo-ros-pkgs / ros-noetic-gazebo-plugins).

If topic exists but different name, remap or change subscriber.

OpenCV windows not visible

If running headless or over SSH without X, windows won’t appear. Use rqt_image_view to inspect topics:

rosrun rqt_image_view rqt_image_view

Plugin fails to load

Install missing packages:

sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins

Tuning parameters (inside scripts/line_follower.py)

Kp (proportional gain) — default ~0.003. Try 0.001–0.01.

Kd (derivative gain) — default ~0.001. Small value.

normal_linear_speed — 0.2–0.4

turn_angular_speed & turn_duration — tune for a clean 90° left turn.

error_threshold & required_consecutive — tune junction detection sensitivity

max_search_time — how long to search after completing a turn

Edit, save, rebuild (if necessary), and relaunch.

Common edits you may want

Change camera ROI, thresholding, or color detection to match line color.

Replace simple threshold with HSV color segmentation for colored tracks.

Add logging to /rosout instead of only stdout for persistent debugging.

Example remap (if camera topic differs)

Add inside your launch file where line_follower node is declared:

<node name="line_follower" pkg="turtle_line_follower" type="line_follower.py" output="screen">
  <remap from="/turtle_camera/image_raw" to="/camera/image_raw"/>
</node>
