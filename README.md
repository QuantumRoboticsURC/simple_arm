## Description

A simple robot drive system that includes skid steering joystick teleoperation, control of a panning two servos to look around the robot, and Arduino firmware.

##
![controller](https://raw.githubusercontent.com/QuantumRoboticsURC/simple_drive/main/images/simple_drive_controller.png)

## Quick Start

1. Install:

```
$ cd catkin/ws
$ git clone https://github.com/QuantumRoboticsURC/simple_drive.git
```

2. Launch ROS nodes:

```
$ roslaunch simple_drive drive_teleop.launch joy_dev:=/dev/input/js1
$ roslaunch simple_drive cmd_vel_mux.launch
$ roslaunch simple_drive simple_drive.launch serial_dev:=/dev/ttyUSB0
```

3. Install the drive_firmware onto a microcontroller connected to servo motors:

```
$

```

4. Install the Servo library and run the code:

```
$ roscd simple_drive
$ cd ./drive_firmware/
$ pio init --board uno
$ pio lib install "arduino-libraries/Servo@^1.1.7"
$ pio run --target upload
```


