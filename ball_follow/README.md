# ball_follow Layer

## Overview

The `ball_follow` layer is a ROS 2-based system designed for detecting, tracking, and following balls using a robotic system equipped with a camera. 
This layer is part of a larger robotics project and provides the functionality to autonomously identify and pursue balls in the robot's environment.

## Components

The layer consists of four main modules:

1. **Camera** (`camera.py`): Captures and publishes raw images from the camera.
2. **Detect Ball** (`detect_ball.py`): Processes images to detect balls and publish their information.
3. **Process Image** (`process_image.py`): Provides image processing utilities for ball detection.
4. **Follow Ball** (`follow_ball.py`): Manages the ball following behavior and robot movement.

## Prerequisites

- ROS 2 (tested with Foxy Fitzroy)
- Python 3.8+
- OpenCV
- NumPy
- Additional ROS 2 packages: `sensor_msgs`, `std_msgs`, `cv_bridge`, `fetchbot_interfaces`

## Usage

To launch the entire `ball_follow` layer:

```
ros2 launch ball_follow ball_follow.launch.py
```

This will start all necessary nodes for ball detection, tracking, and following.

## Nodes and Topics

### Nodes

- `camera_node`: Publishes raw camera images
- `detect_ball`: Detects balls in the images
- `follow_ball`: Manages the ball following behavior

### Topics

- `/camera_sensor/image_raw` (sensor_msgs/Image): Raw camera images
- `/detected_ball` (fetchbot_interfaces/BallInfo): Information about detected balls
- `/follow_ball` (fetchbot_interfaces/Heading): Movement commands for the robot
- `/follow_ball/status` (std_msgs/String): Current status of the ball follower

## Configuration

Key parameters can be adjusted in the respective node's parameters:

### Camera Node
- `camera_device`: Path to the camera device
- `image_width`, `image_height`: Image resolution
- `fps`: Frame rate

### Detect Ball Node
- `tuning_mode`: Enable/disable tuning mode for ball detection
- Various HSV and size thresholds for ball detection

### Follow Ball Node
- `rcv_timeout_secs`: Timeout for receiving ball detection
- `angular_chase_multiplier`, `forward_chase_speed`: Movement parameters
- `search_angular_speed`, `search_rotations`: Search behavior parameters

## State Machine

The Follow Ball node implements a state machine with the following states:
- WAITING: Waiting for a ball to be detected
- FOLLOWING: Actively following a detected ball
- SEARCHING: Searching for a ball when it's lost
- UNREACHABLE: Ball detected but cannot be reached

Refer to the source code for detailed state transition logic.

## Testing

Run the test suite with:

```
colcon test --packages-select ball_follow
```

Refer to the `test/` directory for specific test files and scenarios.

## Troubleshooting

Common issues and their solutions:

1. **Camera not detected**: Check the `camera_device` parameter and ensure the camera is properly connected.
2. **Ball detection issues**: Adjust the HSV and size thresholds in the Detect Ball node parameters.
3. **Robot not moving**: Verify that the Follow Ball node is receiving ball detection information and publishing movement commands.

