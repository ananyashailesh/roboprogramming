# TurtleBot ROS Line Follower with T-Junction Detection

A complete ROS package for a TurtleBot that follows a black line using camera vision and intelligently detects and navigates T-junctions by turning left onto branch paths.

## Features

- 🤖 **Line Following**: PD controller for smooth line tracking
- 🔍 **T-Junction Detection**: Automatically detects intersections using error-based logic
- 🔄 **Autonomous Navigation**: Executes left turns and searches for new paths
- 📹 **Vision-based**: Uses camera feed with OpenCV for line detection
- 🎮 **State Machine**: Robust multi-state control system
- 📊 **Real-time Visualization**: Live display of detection state and errors

## System Requirements

- Ubuntu 20.04 (or compatible)
- ROS Noetic
- Gazebo 11
- Python 3
- OpenCV

## Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport \
ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control python3-opencv
```

## Installation

### 1. Create Workspace

```bash
mkdir -p ~/turtle_line_ws/src
cd ~/turtle_line_ws
catkin_make
echo "source ~/turtle_line_ws/devel/setup.bash" >> ~/.bashrc
source ~/turtle_line_ws/devel/setup.bash
```

### 2. Create Package

```bash
cd ~/turtle_line_ws/src
catkin_create_pkg turtle_line_follower rospy std_msgs sensor_msgs geometry_msgs \
cv_bridge image_transport robot_state_publisher tf gazebo_ros
```

### 3. Create Directory Structure

```bash
cd ~/turtle_line_ws/src/turtle_line_follower
mkdir -p urdf worlds launch scripts
```

### 4. Add Robot URDF

Create `~/turtle_line_ws/src/turtle_line_follower/urdf/turtle_bot.urdf` with the robot definition including:
- Differential drive base
- Two wheels with continuous joints
- Front-facing camera sensor
- Gazebo plugins for control and sensing

### 5. Create Gazebo World

Create `~/turtle_line_ws/src/turtle_line_follower/worlds/line_track.world` with:
- Black line track (6m long, 0.1m wide)
- T-junction with left branch
- Ground plane and lighting

### 6. Setup Launch File

Create `~/turtle_line_ws/src/turtle_line_follower/launch/turtle_line_follower.launch` to:
- Start Gazebo with custom world
- Spawn robot at starting position (-2.5, 0, 0.1)
- Launch robot state publisher
- Run line follower node

### 7. Add Line Follower Script

Create `~/turtle_line_ws/src/turtle_line_follower/scripts/line_follower.py` and make it executable:

```bash
chmod +x ~/turtle_line_ws/src/turtle_line_follower/scripts/line_follower.py
```

### 8. Build Package

```bash
cd ~/turtle_line_ws
catkin_make
source devel/setup.bash
```

## Usage

Launch the complete simulation:

```bash
roslaunch turtle_line_follower turtle_line_follower.launch
```

This will:
1. Open Gazebo with the line track world
2. Spawn the TurtleBot at the starting position
3. Start the line follower node
4. Display a visualization window showing line detection

## How It Works

### State Machine

The robot operates using a finite state machine with five states:

1. **FOLLOWING**: Normal line following using PD control
2. **JUNCTION_DETECTED**: T-junction identified, preparing to turn
3. **TURNING**: Executing a timed left turn
4. **SEARCHING**: Moving forward to find the new line
5. **FOLLOWING_NEW_LINE**: Following the branch path

### Junction Detection Algorithm

- Monitors line centroid error continuously
- Detects junction when error exceeds threshold (>15 pixels) for 3 consecutive frames
- High error indicates the line "splits" - a clear junction signature

### Control Parameters

```python
error_threshold = 15           # Pixels for junction detection
required_consecutive = 3       # Frames needed to confirm junction
normal_linear_speed = 0.3      # m/s during line following
turn_angular_speed = 0.6       # rad/s during turn
turn_duration = 1.2            # seconds for 90° left turn
Kp = 0.003                     # Proportional gain
Kd = 0.001                     # Derivative gain
```

## Tuning Guide

### If robot overshoots turns:
- Reduce `turn_duration`
- Decrease `turn_angular_speed`

### If robot misses the line after turning:
- Increase `max_search_time`
- Adjust `turn_duration` for better alignment

### For smoother line following:
- Tune `Kp` (increase for faster correction)
- Tune `Kd` (increase for smoother motion)

### For better junction detection:
- Adjust `error_threshold` based on line width
- Modify `required_consecutive` for stability

## Topics

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Subscribed
- `/turtle_camera/image_raw` (sensor_msgs/Image): Camera feed

## Sample Output

```
[ INFO]: Line follower started. Press Ctrl+C to stop.
[ INFO]: Following line - Error: -2.1, Angular: 0.006
[ INFO]: Following line - Error: 1.8, Angular: -0.005
[ INFO]: High error detected: 18, consecutive: 1
[ INFO]: High error detected: 22, consecutive: 2
[ INFO]: High error detected: 19, consecutive: 3
[ INFO]: JUNCTION DETECTED! Initiating left turn sequence
[ INFO]: Junction confirmed. Stopping before turn...
[ INFO]: Starting LEFT TURN...
[ INFO]: Turning left... 0.5/1.2s
[ INFO]: Turn completed. Searching for left branch line...
[ INFO]: Found left branch line! Error: -7
[ INFO]: Following new line - Error: -4.8
```

## Troubleshooting

### Gazebo crashes or won't start
```bash
killall gzserver gzclient
roslaunch turtle_line_follower turtle_line_follower.launch
```

### No camera image
- Check topic: `rostopic echo /turtle_camera/image_raw`
- Verify camera plugin in URDF is properly configured

### Robot doesn't move
- Check velocity commands: `rostopic echo /cmd_vel`
- Verify differential drive plugin is loaded

### Line not detected
- Adjust lighting in Gazebo world
- Modify threshold value in `process_image()` function
- Ensure line contrast is sufficient

## Project Structure

```
turtle_line_ws/
├── src/
│   └── turtle_line_follower/
│       ├── urdf/
│       │   └── turtle_bot.urdf
│       ├── worlds/
│       │   └── line_track.world
│       ├── launch/
│       │   └── turtle_line_follower.launch
│       ├── scripts/
│       │   └── line_follower.py
│       ├── CMakeLists.txt
│       └── package.xml
└── devel/
```

## Future Enhancements

- [ ] Add right turn capability
- [ ] Support for multiple junction types
- [ ] Implement line color detection
- [ ] Add obstacle avoidance
- [ ] Create more complex track layouts
- [ ] Add machine learning-based path decision making

## License

MIT License - Feel free to use and modify for your projects.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.



## Acknowledgments

- ROS community for excellent documentation
- Gazebo simulator team
- OpenCV library contributors