# Serial Motor Demo - ROS2 Jazzy Compatible

This is a fixed version of the serial motor demo package, compatible with ROS2 Jazzy. The original code has been updated to work with the latest ROS2 Jazzy distribution.

## Overview

This package provides a ROS2 interface for controlling motors via serial communication with an Arduino microcontroller. It's designed to work with the ROSArduinoBridge firmware.

## Key Features

- **ROS2 Jazzy Compatible**: Updated from deprecated ROS2 APIs
- **Serial Communication**: Communicates with Arduino via USB serial
- **Motor Control**: Supports both PWM and feedback control modes
- **Encoder Support**: Reads encoder values for closed-loop control
- **Timer-based Architecture**: Uses proper ROS2 timers instead of polling loops

## Hardware Requirements

- Arduino Nano (or compatible) running ROSArduinoBridge firmware
- Motor controllers (TB6612FNG or similar)
- Rotary encoders (1860 CPR configured by default)
- USB serial connection to Raspberry Pi

## Installation

1. Clone this repository to your ROS2 workspace:
```bash
cd ~/your_ros2_ws/src
git clone https://github.com/yuanroets/serial_motor_demo_fixed.git
```

2. Install dependencies:
```bash
sudo apt install python3-serial
```

3. Build the package:
```bash
cd ~/your_ros2_ws
colcon build --packages-select serial_motor_demo_msgs serial_motor_demo
source install/setup.bash
```

## Configuration

Default parameters:
- **Encoder CPR**: 1860 (configurable)
- **Serial Port**: `/dev/ttyUSB0` (configurable)
- **Baud Rate**: 57600 (configurable)
- **Loop Rate**: 30 Hz (configurable)

## Usage

### Running the Driver

```bash
ros2 run serial_motor_demo driver
```

### With Custom Parameters

```bash
ros2 run serial_motor_demo driver --ros-args \
  -p encoder_cpr:=1860 \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=57600 \
  -p loop_rate:=30.0 \
  -p serial_debug:=true
```

### Topics

- **Subscribed Topics**:
  - `/motor_command` (serial_motor_demo_msgs/MotorCommand): Motor control commands

- **Published Topics**:
  - `/motor_vels` (serial_motor_demo_msgs/MotorVels): Motor velocities
  - `/encoder_vals` (serial_motor_demo_msgs/EncoderVals): Encoder values

### Message Types

**MotorCommand**:
```
bool is_pwm                    # True for PWM mode, false for feedback mode
float32 mot_1_req_rad_sec      # Motor 1 command (PWM value or rad/sec)
float32 mot_2_req_rad_sec      # Motor 2 command (PWM value or rad/sec)
```

**MotorVels**:
```
float32 mot_1_rad_sec          # Motor 1 velocity in rad/sec
float32 mot_2_rad_sec          # Motor 2 velocity in rad/sec
```

**EncoderVals**:
```
int32 mot_1_enc_val           # Motor 1 encoder count
int32 mot_2_enc_val           # Motor 2 encoder count
```

## Changes from Original

### Fixed Issues:
1. **Corrupted Message Files**: Removed markdown artifacts from .msg files
2. **Deprecated create_rate()**: Replaced with proper timer-based architecture
3. **Main Loop Structure**: Updated to use `rclpy.spin()` instead of manual polling
4. **Dependencies**: Added python3-serial dependency
5. **Default Parameters**: Set sensible defaults (CPR=1860, rate=30Hz)

### Compatibility:
- ✅ ROS2 Jazzy Jalisco
- ✅ Ubuntu 24.04 Noble
- ✅ Python 3.12

## Arduino Firmware

This package is designed to work with the ROSArduinoBridge firmware. Make sure your Arduino is programmed with compatible firmware that responds to these serial commands:

- `o <pwm1> <pwm2>`: Set PWM values
- `m <count1> <count2>`: Set encoder counts per loop for feedback control
- `e`: Read encoder values

## License

[Original license applies]

## Contributing

Feel free to submit issues and pull requests to improve this package.

## Credits

Based on the original serial_motor_demo by Josh Newans, updated for ROS2 Jazzy compatibility.
