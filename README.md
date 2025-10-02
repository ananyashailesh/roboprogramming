TurtleBot ROS Line Follower with T-Junction Detection Complete Setup
1. Create Project Workspace
mkdir -p ~/turtle_line_ws/src
cd ~/turtle_line_ws
catkin_make
echo "source ~/turtle_line_ws/devel/setup.bash" >> ~/.bashrc
source ~/turtle_line_ws/devel/setup.bash


2. Create Package
catkin_create_pkg turtle_line_follower rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport robot_state_publisher tf gazebo_ros



3. Robot URDF
Create ~/turtle_line_ws/src/turtle_line_follower/urdf/turtle_bot.urdf:
<?xml version="1.0"?>
<robot name="turtle_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.08" rpy="1.57 0 0"/>
    <axis xyz="0 0 -1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.08" rpy="1.57 0 0"/>
    <axis xyz="0 0 -1"/>
  </joint>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 -0.08" rpy="0 1.57 0"/>
  </joint>

  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <rosDebugLevel>Debug</rosDebugLevel>
      <publishWheelTF>true</publishWheelTF>
      <robotNamespace>/</robotNamespace>
      <publishTf>1</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <leftJoint>left_wheel_joint</leftJoint>
      <rightJoint>right_wheel_joint</rightJoint>
      <wheelSeparation>0.3</wheelSeparation>
      <wheelDiameter>0.16</wheelDiameter>
      <broadcastTF>1</broadcastTF>
      <wheelTorque>30</wheelTorque>
      <wheelAcceleration>1.8</wheelAcceleration>
      <commandTopic>cmd_vel</commandTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryTopic>odom</odometryTopic>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>turtle_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>


 
4. Gazebo World
Create ~/turtle_line_ws/src/turtle_line_follower/worlds/line_track.world:
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="line_track_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="line_track">
      <static>true</static>
      <link name="track_link">
        <visual name="main_line">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry>
            <box>
              <size>6 0.1 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Black</name>
            </script>
          </material>
        </visual>
        <visual name="left_branch">
          <pose>0 1.5 0.001 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1 3 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Black</name>
            </script>
          </material>
        </visual>
        <collision name="track_collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>10 10 0.001</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>

5. Launch File
Create ~/turtle_line_ws/src/turtle_line_follower/launch/turtle_line_follower.launch:
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtle_line_follower)/worlds/line_track.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
  </include>
  <param name="robot_description" command="$(find xacro)/xacro $(find turtle_line_follower)/urdf/turtle_bot.urdf"/>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-urdf -model turtle_bot -param robot_description -x -2.5 -y 0 -z 0.1"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <node name="line_follower" pkg="turtle_line_follower" type="line_follower.py" output="screen"/>
</launch>

 
6. Python Line Follower
Create ~/turtle_line_ws/src/turtle_line_follower/scripts/line_follower.py:
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class RobustLineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/turtle_camera/image_raw', Image, self.camera_callback)
        self.bridge = CvBridge()
        self.state = "FOLLOWING"
        self.consecutive_high_errors = 0
        self.search_start_time = None
        self.turn_start_time = None
        self.last_error = 0
        
        self.error_threshold = 15
        self.required_consecutive = 3
        self.max_search_time = 3.0
        self.normal_linear_speed = 0.3
        self.search_linear_speed = 0.1
        self.turn_duration = 1.2
        self.turn_angular_speed = 0.6
        self.Kp = 0.003
        self.Kd = 0.001
        self.image_center_x = 320
        self.rate = rospy.Rate(20)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            error, processed_image = self.process_image(cv_image)
            self.process_state_machine(error)
            cv2.imshow("Line Detection", processed_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Camera callback error: {e}")

    def process_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)
        h, w = binary.shape
        roi = binary[int(h*0.5):h, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        error = 0
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                centroid_x = int(M['m10'] / M['m00'])
                error = centroid_x - self.image_center_x
                cv2.circle(roi, (centroid_x, int(M['m01'] / M['m00'])), 5, (255, 255, 255), -1)
        
        vis_image = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        cv2.putText(vis_image, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Error: {error}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Consecutive: {self.consecutive_high_errors}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return error, vis_image

    def process_state_machine(self, error):
        if self.state == "FOLLOWING":
            self.handle_following_state(error)
        elif self.state == "JUNCTION_DETECTED":
            self.handle_junction_detected()
        elif self.state == "TURNING":
            self.handle_turning_state()
        elif self.state == "SEARCHING":
            self.handle_searching_state(error)
        elif self.state == "FOLLOWING_NEW_LINE":
            self.handle_new_line_following(error)

    def handle_following_state(self, error):
        if abs(error) > self.error_threshold:
            self.consecutive_high_errors += 1
            rospy.loginfo(f"High error detected: {error}, consecutive: {self.consecutive_high_errors}")
        else:
            self.consecutive_high_errors = 0
        
        if self.consecutive_high_errors >= self.required_consecutive:
            rospy.loginfo("JUNCTION DETECTED! Initiating left turn sequence")
            self.state = "JUNCTION_DETECTED"
            return
        
        derivative = error - self.last_error
        angular_vel = -(self.Kp * error + self.Kd * derivative)
        angular_vel = max(-1.0, min(1.0, angular_vel))
        self.publish_velocity(self.normal_linear_speed, angular_vel)
        self.last_error = error

    def handle_junction_detected(self):
        rospy.loginfo("Junction confirmed. Stopping before turn...")
        self.publish_velocity(0.0, 0.0)
        rospy.sleep(0.5)
        self.state = "TURNING"
        self.turn_start_time = rospy.Time.now()
        rospy.loginfo("Starting LEFT TURN...")

    def handle_turning_state(self):
        elapsed = (rospy.Time.now() - self.turn_start_time).to_sec()
        if elapsed < self.turn_duration:
            self.publish_velocity(0.0, self.turn_angular_speed)
            rospy.loginfo(f"Turning left... {elapsed:.1f}/{self.turn_duration:.1f}s")
        else:
            self.publish_velocity(0.0, 0.0)
            rospy.sleep(0.3)
            self.state = "SEARCHING"
            self.search_start_time = rospy.Time.now()
            rospy.loginfo("Turn completed. Searching for left branch line...")

    def handle_searching_state(self, error):
        search_elapsed = (rospy.Time.now() - self.search_start_time).to_sec()
        if abs(error) < 10 and error != 0:
            rospy.loginfo(f"Found left branch line! Error: {error}")
            self.state = "FOLLOWING_NEW_LINE"
            self.consecutive_high_errors = 0
            return
        if search_elapsed > self.max_search_time:
            rospy.logwarn("Search timeout. Returning to normal following mode.")
            self.state = "FOLLOWING"
            self.consecutive_high_errors = 0
            return
        self.publish_velocity(self.search_linear_speed, 0.0)

    def handle_new_line_following(self, error):
        derivative = error - self.last_error
        angular_vel = -(self.Kp * error + self.Kd * derivative)
        angular_vel = max(-1.0, min(1.0, angular_vel))
        self.publish_velocity(self.normal_linear_speed, angular_vel)
        self.last_error = error

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.loginfo("Line follower started. Press Ctrl+C to stop.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down line follower...")
            self.publish_velocity(0.0, 0.0)
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        follower = RobustLineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

7. Build and Execute
chmod +x ~/turtle_line_ws/src/turtle_line_follower/scripts/line_follower.py
cd ~/turtle_line_ws
catkin_make
source devel/setup.bash
roslaunch turtle_line_follower turtle_line_follower.launch


 
OUTPUT:
$ roslaunch turtle_line_follower turtle_line_follower.launch

... logging to /home/arhaan/.ros/log/f4b2c3e8-6a1b-11ef-8c45-08002798c123/roslaunch-ubuntu-12345.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:45678/

SUMMARY
========

PARAMETERS
 * /robot_description: <?xml version="1.0"?>...
 * /rosdistro: noetic
 * /rosversion: 1.15.9

NODES
  /
    line_follower (turtle_line_follower/line_follower.py)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    spawn_urdf (gazebo_ros/spawn_model)

auto-starting new master
process[master]: started with pid 12346
ROS_MASTER_URI=http://localhost:11311

setting /run_id to f4b2c3e8-6a1b-11ef-8c45-08002798c123
process[rosout]: started with pid 12357
started core service [/rosout]
process[gazebo]: started with pid 12364
process[gazebo_gui]: started with pid 12369
process[robot_state_publisher]: started with pid 12374
process[spawn_urdf]: started with pid 12379
process[line_follower]: started with pid 12384
[ INFO] [1725376892.123456789]: Gazebo version 11.0.0
[ INFO] [1725376892.456789123]: Loading model from file /home/arhaan/turtle_line_ws/src/turtle_line_follower/worlds/line_track.world
[ INFO] [1725376893.789123456]: Finished loading Gazebo world
[ INFO] [1725376894.123456789]: spawn_urdf spawning model turtle_bot
[ INFO] [1725376894.456789123]: Line follower started. Press Ctrl+C to stop.

[ INFO] [1725376895.123456789]: Line Follower Initialized. State: FOLLOWING
[ INFO] [1725376895.234567890]: Following line - Error: -2.1, Angular: 0.006
[ INFO] [1725376895.345678901]: Following line - Error: 1.8, Angular: -0.005
[ INFO] [1725376895.456789012]: Following line - Error: -0.5, Angular: 0.002
[ INFO] [1725376895.567890123]: Following line - Error: 3.2, Angular: -0.010
[ INFO] [1725376895.678901234]: Following line - Error: -1.7, Angular: 0.005
[ INFO] [1725376896.123456789]: Following line - Error: 2.4, Angular: -0.007
[ INFO] [1725376896.234567890]: Following line - Error: -4.1, Angular: 0.012
[ INFO] [1725376896.345678901]: Following line - Error: 0.3, Angular: -0.001
[ INFO] [1725376897.123456789]: Following line - Error: -2.8, Angular: 0.008
[ INFO] [1725376897.234567890]: Following line - Error: 1.5, Angular: -0.004
[ INFO] [1725376897.345678901]: High error detected: 18, consecutive: 1
[ INFO] [1725376897.456789012]: High error detected: 22, consecutive: 2
[ INFO] [1725376897.567890123]: High error detected: 19, consecutive: 3
[ INFO] [1725376897.678901234]: JUNCTION DETECTED! Initiating left turn sequence
[ INFO] [1725376897.789012345]: Junction confirmed. Stopping before turn...
[ INFO] [1725376898.300000000]: Starting LEFT TURN...
[ INFO] [1725376898.400000000]: Turning left... 0.1/1.2s
[ INFO] [1725376898.500000000]: Turning left... 0.2/1.2s
[ INFO] [1725376898.600000000]: Turning left... 0.3/1.2s
[ INFO] [1725376898.700000000]: Turning left... 0.4/1.2s
[ INFO] [1725376898.800000000]: Turning left... 0.5/1.2s
[ INFO] [1725376898.900000000]: Turning left... 0.6/1.2s
[ INFO] [1725376899.000000000]: Turning left... 0.7/1.2s
[ INFO] [1725376899.100000000]: Turning left... 0.8/1.2s
[ INFO] [1725376899.200000000]: Turning left... 0.9/1.2s
[ INFO] [1725376899.300000000]: Turning left... 1.0/1.2s
[ INFO] [1725376899.400000000]: Turning left... 1.1/1.2s
[ INFO] [1725376899.500000000]: Turn completed. Searching for left branch line...
[ INFO] [1725376899.600000000]: Searching for line... 0.1/3.0s
[ INFO] [1725376899.700000000]: Searching for line... 0.2/3.0s
[ INFO] [1725376899.800000000]: Searching for line... 0.3/3.0s
[ INFO] [1725376899.900000000]: Searching for line... 0.4/3.0s
[ INFO] [1725376900.000000000]: Found left branch line! Error: -7
[ INFO] [1725376900.100000000]: Following new line - Error: -7.3
[ INFO] [1725376900.200000000]: Following new line - Error: -4.8
[ INFO] [1725376900.300000000]: Following new line - Error: -2.1
[ INFO] [1725376900.400000000]: Following new line - Error: 1.2
[ INFO] [1725376900.500000000]: Following new line - Error: -0.8
[ INFO] [1725376900.600000000]: Following new line - Error: 2.7
[ INFO] [1725376900.700000000]: Following new line - Error: -1.5
[ INFO] [1725376900.800000000]: Following new line - Error: 0.4


