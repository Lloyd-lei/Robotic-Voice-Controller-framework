# Robotic Voice Controller Framework

A voice-controlled robotic arm simulation framework using MuJoCo physics engine and OpenAI's Realtime API.

## 📹 Demo

> 🎬 **Demo Video**: [Watch on Bilibili](https://bilibili.com/video/BV1UfxpzaE2F/)

We are StarbotUSA, a startup robotics company in Santa Barbara. This project is aimed to take AI embedding on the robot. The code here is more of an experimental tryout but you can imagine that robots are going to interact with human beings in a very revolutionary way (Probably we don't need the smart phone to interact with everything very soon). In the future, we will implement this function in ROS2 of the real Unitree G1 robot. Please follow us. If you are interested in robotics and would like to share codes, like AI embedding, ROS2, IV algr, vision perception, locomotion, and manipulation, please contact me lloyd@starbotusa.com. We will be happy to pay or collaborate with you!!!

## Installation

### Prerequisites

```bash
pip install mujoco
pip install numpy
```

### Optional Dependencies

For voice control:
```bash
pip install websockets pyaudio python-dotenv
```

For IK solver:
```bash
pip install pinocchio
```

For keyboard control:
```bash
pip install keyboard
```

For visualization:
```bash
pip install mujoco-viewer  # If built-in viewer not available
```

##  Usage

### Basic Usage

```bash
python test_ik.py
```

### Voice Control Setup

1. Create a `.env` file in the project root
2. Add your OpenAI API key:
   ```
   OPENAI_API_KEY=your_api_key_here
   ```
3. Run the script and start speaking commands like:
   - "Move forward"
   - "Go up"
   - "Rotate left"
   - "Reset position"

### Keyboard Controls

- **Arrow Keys**: Move forward/backward/left/right
- **Page Up/Down**: Move up/down
- **Shift + Arrow Keys**: Rotate around pitch/yaw axes
- **Shift + Page Up/Down**: Rotate around roll axis
- **ESC**: Exit simulation

## 📁 Project Structure

```
.
├── test_ik.py              # Main simulation script
├── robot_arm_ik.py         # IK solver implementation
├── weighted_moving_filter.py # Filter utilities
├── assets/                 # Robot models and meshes
│   ├── g1_29dof_rev_1_0.xml
│   └── meshes/
├── unitree_ros/            # Unitree ROS packages
└── xr_teleoperate/         # XR teleoperation modules
```

## 🤖 Supported Commands

### Movement Commands
- `move forward [normal/large]`
- `move backward [normal/large]`
- `move left [normal/large]`
- `move right [normal/large]`
- `move up [normal/large]`
- `move down [normal/large]`

### Rotation Commands
- `rotate roll [normal/large]`
- `rotate pitch [normal/large]`
- `rotate yaw [normal/large]`

### Utility Commands
- `reset hand` - Reset to initial position

## 🔧 Configuration

### Position Control
- `position_step`: 0.01 (keyboard)
- `voice_position_step`: 0.05 (voice)

### Rotation Control
- `rotation_step`: 0.05 (keyboard)
- `voice_rotation_step`: 0.2 (voice)

##  Requirements

- Python 3.8+
- MuJoCo 2.3+
- OpenAI API key (for voice control)
- Audio input device (for voice control)

##  Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

##  License

This project includes components from Unitree Robotics. Please refer to individual component licenses.

