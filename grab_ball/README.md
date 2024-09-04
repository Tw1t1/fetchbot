# grab_ball Layer

## Overview

The `grab_ball` layer is a ROS 2-based system designed for detecting, tracking, and grabbing balls using a robotic claw mechanism. This layer is part of a larger robotics project and provides the functionality to autonomously identify and capture balls in the robot's environment.

## Components

The layer consists of three main modules:

1. **Ball Grabber** (`ball_grabber.py`): Manages the ball detection and grabbing process.
2. **Claw Controller** (`claw_controller.py`): Controls the claw mechanism.
3. **Hardware Interface** (`hardware.py`): Provides low-level hardware control for motors and sensors.

![Claw Mechanism](claw_mechanism.jpg)
*Figure 1: Close-up view of the claw mechanism, showing the VEX potentiometer and structural details.*

## Hardware Specifications

- **Robotic Arm**: VEX claw arm
- **Position Sensor**: VEX potentiometer connected to the arm motor
- **Analog-to-Digital Converter**: MCP3002 for reading potentiometer values
- **Motor Driver**: L298N for controlling the DC motor of the arm


![Ball Gripping](ball_gripping.jpg)
*Figure 2: The claw mechanism gripping a blue ball, with the camera visible above.*

## Prerequisites

- ROS 2 Humble
- Python 3.8+
- Raspberry Pi 4 (for hardware control)
- Additional ROS 2 packages: `std_msgs`, `fetchbot_interfaces`
- Machine Learning libraries: `numpy 1.24.3`, `scikit-learn 1.5.1`, `joblib 1.4.2`

## Usage

To launch the entire `grab_ball` layer:

```
ros2 launch grab_ball grab_ball.launch.py
```

This will start all necessary nodes for ball detection, tracking, and grabbing.

## Nodes and Topics

### Nodes

- `ball_grabber`: Manages the overall ball grabbing process
- `claw_controller`: Controls the claw mechanism
- `ball_detector`: Detects balls in the environment (not part of this layer)

### Topics

- `/detected_ball` (Input): Receives ball detection information
- `grab_ball/status` (Output): Publishes the current status of the grabbing process
- `claw_cmd` (Internal): Sends commands to the claw controller
- `position` (Internal): Reports the current position of the claw

## Machine Learning Models

The `ball_grabber` node uses two pre-trained machine learning models:

1. **Graspable Classifier**: Determines if a detected ball is graspable based on its features.
2. **Distance Estimator**: Estimates the distance to the ball using various regression models.

These models are loaded from the `distance_models` directory and are used to make decisions about when to attempt grabbing a ball.

### Model Training

The models were trained using a custom calibration script (`distance_calibration.py`) see in `distance_models` directory. Key details about the training process:

- **Data Collection**: Approximately 1600 samples were collected, capturing ball size, position (X and Y), distance, and graspability.
- **Features**: The models use three features: ball size, X position, and Y position.
- **Target Variables**: 
  - Distance (in cm) for regression models
  - Graspability (binary) for the classification model

![Data Collection Setup](data_collection.jpg)
*Figure 3: Setup for collecting training data for machine learning models, showing the gridded surface for accurate measurements.*

### Model Types

Several regression models were trained for distance estimation:

1. Linear Regression
2. Polynomial Regression (degree 2)
3. Random Forest Regression
4. Neural Network Regression (MLP)

For graspability classification:

- Gradient Boosting Classifier

### Model Selection

The `ball_grabber` node uses the Random Forest model for distance estimation due to its balance of accuracy and interpretability. The Gradient Boosting Classifier is used for determining graspability.

### Performance Metrics

Model performance was evaluated using:

- Mean Squared Error (MSE) and R-squared (R2) for regression models
- Accuracy for the classification model

Exact performance metrics may vary based on the specific training run.

### Dynamic Thresholding

The system implements a dynamic thresholding mechanism to adjust graspability predictions based on the ball's position in the frame. This helps account for the varying difficulty of grasping balls at different locations.

## Configuration

Key parameters can be adjusted in the `config/grab_ball_params.yaml` file:

- `position_change_threshold`: Threshold for detecting claw position changes
- `position_range_min`: Minimum acceptable claw position
- `position_range_max`: Maximum acceptable claw position

## Troubleshooting

Common issues and their solutions:

1. **Claw not responding**: Check hardware connections, especially the L298N motor driver connections.
2. **Inconsistent position readings**: Verify the MCP3002 ADC connections and the VEX potentiometer attachment.
3. **Ball detection issues**: Ensure the `ball_detector` node is functioning correctly and publishing to the `/detected_ball` topic.
4. **ROS communication errors**: Confirm all nodes are on the same ROS network and can communicate.
5. **Machine learning model errors**: Check that the model files are present in the `distance_models` directory and are compatible with the installed scikit-learn version.


