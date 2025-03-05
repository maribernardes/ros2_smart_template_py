# 3DOF Smart Template (ROS2) python package: ros2_smart_template_py

## Quick demo test
To give it a try on simulation mode, launch:
```
ros2 launch smart_template_py robot.launch.py sim_level:=1 rviz:=true
```

## Overview
This repository contains:

- ROS2 package for communicating with the 3 DoF Smart Template
- The 3DoF Smart Template uses Galil Controller to control the position of the 3 motors
- ROS2 Foxy, Ubuntu 20.04 / ROS2 Humble, Ubuntu 22.04

## Description
### Subscribers
- None

### Publishers
- '/stage/state/guide_pose', a PoseStamped with the current position of the template. It's refresh rate is given by the "timer_period"
- '/joint_states', a JointState with the current joint values of the template. It's refresh rate is given by the "timer_period"

### Action server

- The template node exposes the /stage/move_and_observe action which takes a /smart_template_interfaces/action/MoveAndObserve action message of the format:
```
float64 x
float64 y
float64 z
float64 eps
---
float64 x
float64 y
float64 z
float64 time
float64 error
int32 error_code
---
float64 x
float64 y
float64 z
float64 error
float64 time
```

### Service server

- The template node exposes the /stage/command service which takes a /smart_template_interfaces/srv/Command action message of the format:
```
string command
---
string response
```
where command can be: 'HOME', or 'RETRACT' ('ABORT' still to be implemented)

- The template node exposes the /stage/get_position service which takes a /smart_template_interfaces/srv/GetPoint action message of the format:
```
---
float64 x
float64 y
float64 z
bool valid
```
where valid is False in case of communication error with Galil, and True otherwise

### Launch files
- robot.launch.py
  * Argument: "sim_level"
    * 1 - VIRTUAL ROBOT (FOR SIMULATION ONLY)
    * 2 - REAL HARDWARE 
  * Argument: "rviz"
    * false - NO rviz
    * true - loads rviz
  * Argument: "gui"
    * false - NO rqt GUI custom plugin
    * true - loads rqt GUI custom plugin

## Usage <a name="usage"></a>
## Axis
- Left-right (horizontal): x
- Anterior-posterior (vertical): z
- Inferior-superior (insertion): y

### Network connection
- Setup the computer network to:
  * IP: 192.168.0.9
  * Subnet mask: 255.255.255.0
 
- Galil commands using gclib:
  * galil = gclib.py() #create communication 
  * galil.GOpen('192.168.0.99') #open communication
  * galil.GCommand('XXXX') #Send command to Galil
  * list of commands:
    - DPX=Y Define the position Y in the axis X
    - PTX Define the absolute motion mode in the axis X
    - PAX=Y Send command to move the axis X to the position Y
    - For more commands, check Galil docummentation 
