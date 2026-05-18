# Robotics Drawing Project - Group 21

## Overview

This is a 4-axis robotic drawing system designed to autonomously draw on a 2D surface. The project combines hardware control via Arduino, computational geometry through inverse kinematics, and a ROS-based simulation environment. The robot uses a rail system (X-axis), arm movement (Y-axis), vertical lift (Z-axis), and rotational control (A-axis) to position a pen and draw patterns.

## Project Structure

```
Robotics_Drawing_project_group_21/
├── README.md                              # This file
├── drawingv3.py                           # Main control script - H-bridge gcode interpreter
├── IK_robotv2.py                          # Inverse kinematics calculations
├── finalcodearduino.ino                   # Arduino firmware for motor control
├── catkin_ws/                             # ROS workspace
│   ├── src/
│   │   ├── URDF_Assembly/                 # Robot URDF model and ROS package
│   │   │   ├── urdf/                      # URDF files for simulation
│   │   │   ├── meshes/                    # 3D mesh files
│   │   │   ├── config/                    # Configuration files
│   │   │   ├── launch/                    # ROS launch files
│   │   │   └── scripts/                   # Helper scripts
│   │   └── CMakeLists.txt
│   ├── build/                             # Build output directory
│   └── devel/                             # Development setup
├── Robotics Final Report.pdf              # Project documentation
├── Black and White Modern Technology Presentation.pdf  # Presentation slides
├── Screencast videos/                     # Demo recordings
└── Images/                                # Project photos and diagrams
```

## Core Components

### 1. **drawingv3.py** - Main Control Script
The primary Python script that communicates with the Arduino to execute drawing commands.

**Key Features:**
- Serial communication with Arduino at 115200 baud
- Gcode-like command interpretation (X, Y, Z, A coordinates)
- Pen height control (PEN_UP_Z = 18.8 cm for travel, PEN_DOWN_Z = 17 cm for drawing)
- Robot workspace limits:
  - X-axis (Rail): 24 cm max
  - Y-axis (Arm): 33.8 cm max
  - Z-axis (Lift): 27.3 cm max
- Motor control via inverse kinematics
- Startup sequence and homing procedures

**Usage:**
```bash
python drawingv3.py
```

### 2. **IK_robotv2.py** - Inverse Kinematics Module
Handles coordinate transformations and motion planning.

**Key Functions:**
- `calculate_smart_move()`: Converts Cartesian coordinates to motor positions
- `home_robot()`: Homes all axes to zero position
- Angle calculations for 4-axis positioning
- Physical constraint validation

**Parameters:**
- ARM_MIN_Y: 3.8 cm (minimum arm extension)
- ARM_MAX_Y: 33.8 cm (maximum arm extension)
- RAIL_MAX_X: 26 cm (rail travel distance)

### 3. **finalcodearduino.ino** - Arduino Firmware
Firmware for Arduino microcontroller managing stepper motors and sensors.

**Hardware Components:**
- **4 Stepper Motors** (AccelStepper library):
  - X-axis: Rail movement (pins 2, 5)
  - Y-axis: Arm extension (pins 3, 6)
  - Z-axis: Vertical lift (pins 4, 7)
  - A-axis: Rotation (pins 12, 13)
- **Stepper Calibration:**
  - X: 100 steps/cm
  - Y: 160 steps/cm
  - Z: 80 steps/cm
  - A: 71.11 steps/degree
- **I2C LCD Display** (0x27 address): Status display
- **Emergency Stop** (E-Stop): Pin A0

**Features:**
- Serial command processing
- Motor enable/disable
- Safety E-stop functionality
- LCD status feedback
- Acceleration/deceleration control

### 4. **URDF Assembly (ROS Package)**
Complete robot model for Gazebo simulation and visualization.

**Contents:**
- `URDF_Assembly.urdf`: Robot description file
- 3D mesh files for visualization
- Launch files for ROS Gazebo simulation
- Configuration files for DH parameters

## Getting Started

### Prerequisites
- Python 3.x
- PySerial library: `pip install pyserial`
- Arduino IDE (for uploading firmware)
- ROS (for simulation - optional)
- CatKin tools (for ROS package management)

### Hardware Setup
1. Connect Arduino to computer via USB
2. Note the COM port (Windows) or device path (Linux/Mac)
3. Update `ARDUINO_PORT` in the Python scripts accordingly
4. Wire stepper motors and control pins per schematic
5. Connect E-stop button to pin A0
6. Install I2C LCD display at address 0x27

### Software Installation

**1. Upload Arduino Firmware:**
```bash
# Open finalcodearduino.ino in Arduino IDE
# Select appropriate board and COM port
# Click Upload
```

**2. Install Python Dependencies:**
```bash
pip install pyserial
```

**3. Build ROS Package (Optional):**
```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### Running the Robot

**Option 1: Direct Control**
```bash
python drawingv3.py
```
The robot will:
1. Connect to Arduino
2. Run startup sequence
3. Home all axes
4. Wait for drawing commands or interactive input

**Option 2: ROS Simulation**
```bash
cd catkin_ws
source devel/setup.bash
roslaunch URDF_Assembly display.launch  # For visualization
# or
roslaunch URDF_Assembly gazebo.launch   # For simulation
```

## Command Format

Commands follow a simple coordinate format:
```
X<value> Y<value> Z<value> A<value>
```
Example: `X10.5 Y15.2 Z18.8 A45.0\n`

- **X**: X-rail position (0-24 cm)
- **Y**: Y-arm extension (0-33.8 cm)
- **Z**: Z-lift height (0-27.3 cm)
- **A**: Rotation angle (0-360 degrees)

## Important Parameters

### Robot Workspace Configuration
```python
X_RAIL_MAX = 24.0 cm      # Maximum X travel distance
Y_ARM_MAX = 33.8 cm       # Maximum arm extension
Z_MAX = 27.3 cm           # Maximum Z height
Y_OFFSET = 13.0 cm        # Paper Y-offset from robot origin
```

### Pen Heights
```python
PEN_UP_Z = 18.8 cm        # Raised position (travel height)
PEN_DOWN_Z = 17 cm        # Lowered position (drawing contact)
```

### Arduino Communication
```
BAUD_RATE = 115200        # Serial communication speed
ESTOP_PIN = A0            # Emergency stop button pin
```

## Calibration

### Motor Step Calibration
If the robot movement seems off, recalibrate the steps per unit:

1. **X-axis (Rail):** Move a known distance and count steps
   - Default: 100 steps/cm
   - Adjust `STEPS_X` in Arduino code

2. **Y-axis (Arm):** Similar process
   - Default: 160 steps/cm
   - Adjust `STEPS_Y`

3. **Z-axis (Lift):** Vertical movement calibration
   - Default: 80 steps/cm
   - Adjust `STEPS_Z`

4. **A-axis (Rotation):** Angle-based calibration
   - Default: 71.11 steps/degree
   - Adjust `STEPS_A`

After changes, upload the new firmware to Arduino.

## Safety Considerations

⚠️ **Important:**
- Always test movements with high pen height first
- Use E-stop feature if erratic behavior occurs
- Never let the arm hit physical limits forcefully
- Ensure paper is properly secured
- Keep fingers clear of moving parts

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Serial connection fails | Check COM port, ensure Arduino drivers installed |
| Motors don't move | Verify Arduino power, check motor connections |
| Pen not drawing | Adjust PEN_DOWN_Z height, check pen contact |
| Jerky movements | Recalibrate motor steps, check for mechanical friction |
| LCD not showing | Verify I2C address (0x27), check connections |
| E-stop doesn't work | Check pin A0 connection, verify pullup resistor |

## Files Description

| File | Purpose |
|------|---------|
| `drawingv3.py` | Main control script with gcode interpreter |
| `IK_robotv2.py` | Inverse kinematics engine |
| `finalcodearduino.ino` | Arduino firmware |
| `Robotics Final Report.pdf` | Complete technical documentation |
| `URDF_Assembly.urdf` | Robot URDF model |
| Demo videos/images | Project demonstrations and photos |

## Team Contributors

Group 21 - Robotics Project

## Additional Resources

- **Final Report:** See `Robotics Final Report.pdf` for detailed technical specifications
- **Presentation:** See `Black and White Modern Technology Presentation.pdf` for project overview
- **Demo Videos:** Multiple screencasts showing robot in action

## License

[Add appropriate license if applicable]

## Contact

For questions or issues related to this project, please refer to the Final Report or contact the project team.

---

**Last Updated:** December 2025
**Status:** Complete and Tested
# Robotics_Drawing_project_group_21
