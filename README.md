# 3DOF Smart Template (ROS2) package: ros2_smart_template

## Overview
This repository contains:

- ROS2 package for communicating with the 3 DoF Smart Template
- The 3DoF Smart Template uses Galil Controller to control the position of the 3 motors
- ROS 2 Galatic, Ubuntu 20.04

## Description
### Subscribers
- '/stage/initial_point', a PoseStamped with the initial position of the smart template

### Publishers
- '/stage/state/guide_pose', a PoseStamped with the current position of the template. It's refresh rate is given by the "timer_period"
### Action server

- The template node exposes the /move_stage action which takes a /stage_control_interfaces/action/MoveStage action message of the format:
  
* float64 x
* float64 y
* float64 z
* float64 eps
* 
* float64 x
* float64 y
* float64 z
* float64 time
* float64 error
* int32 error_code
* 
* float64 x
* float64 y
* float64 z
* float64 error
* float64 time

### Launch files
- robot.launch.py
  * Argument: "sim_level"
    * 1 - VIRTUAL ROBOT (FOR SIMULATION ONLY)
    * 2 - REAL HARDWARE 
## Usage <a name="usage"></a>
## Axis
- Left-right: x
- Anterior-posterior: z
- Inferior-superior: y

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



