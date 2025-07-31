# My Package - ROS2 Webots Robot with OpenAI Vision

This package contains a ROS2 robot simulation using Webots with OpenAI vision capabilities.

## Features

- **Robot Simulation**: A differential drive robot with distance sensors and camera
- **Obstacle Avoidance**: Basic obstacle avoidance using distance sensors
- **OpenAI Vision**: AI-powered vision analysis using OpenAI GPT-4o model
- **RViz Visualization**: Optional RViz integration for visualization

## Prerequisites

1. **OpenAI API Key**: You need an OpenAI API key to use the vision features
2. **Python Dependencies**: Install required Python packages

### Setting up OpenAI API Key

Set your OpenAI API key as an environment variable:

```bash
export OPENAI_API_KEY="your-api-key-here"
```

Or add it to your `.bashrc` file:

```bash
echo 'export OPENAI_API_KEY="your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```

### Installing Python Dependencies

Install the required Python packages:

```bash
pip install openai opencv-python numpy
```

## Building the Package

```bash
cd /path/to/your/ros_ws
colcon build --packages-select my_package
source install/setup.bash
```

## Usage

### Launch with OpenAI Vision (Recommended)

This launches the robot with OpenAI vision analysis:

```bash
ros2 launch my_package openai_api_launch.py
```

### Launch with RViz Visualization

This launches the robot with OpenAI vision and RViz visualization:

```bash
ros2 launch my_package robot_with_rviz_launch.py
```

### Launch Basic Robot

This launches the robot with basic obstacle avoidance only:

```bash
ros2 launch my_package robot_launch.py
```

## How it Works

1. **Camera Input**: The robot's camera captures images from the simulation
2. **Image Processing**: Images are converted to base64 format for API transmission
3. **OpenAI Analysis**: Images are sent to OpenAI GPT-4o with a specific prompt about railroad safety
4. **Response Parsing**: The AI response is parsed for `<GO>` or `<STOP>` commands
5. **Robot Control**: The robot moves forward on `<GO>` or stops on `<STOP>`

## Node Details

### OpenAI API Node (`openai_api`)

- **Subscribes to**: `/camera/image_color` (sensor_msgs/Image)
- **Publishes to**: `/cmd_vel` (geometry_msgs/Twist)
- **Function**: Analyzes camera images using OpenAI API and controls robot movement

### Obstacle Avoider Node (`obstacle_avoider`)

- **Subscribes to**: `/left_sensor`, `/right_sensor` (sensor_msgs/Range)
- **Publishes to**: `/cmd_vel` (geometry_msgs/Twist)
- **Function**: Basic obstacle avoidance using distance sensors

## Configuration

### OpenAI Prompt

The current prompt asks: "What do you see on the rail road? Can i move forward along the rail road safely if i am a tram? If so last your answer with '<GO>' else '<STOP>'"

You can modify this prompt in `my_package/opeai_api.py` in the `process_image()` method.

### Processing Frequency

The node processes images every 2 seconds by default. You can change this in the `__init__()` method by modifying the timer value:

```python
self.timer = self.create_timer(2.0, self.process_image)  # Change 2.0 to desired frequency
```

## Troubleshooting

### Common Issues

1. **OpenAI API Key Error**: Make sure your API key is set correctly
2. **Image Processing Error**: Check that the camera topic is publishing correctly
3. **Import Errors**: Ensure all Python dependencies are installed

### Debugging

Enable debug logging:

```bash
ros2 run my_package openai_api --ros-args --log-level debug
```

## License

This package is licensed under the Apache License 2.0. 