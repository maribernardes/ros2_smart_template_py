# 3DOF Smart Template (ROS2) package: ros2_smart_template

## Overview
This repository contains:

- ROS2 package for communicating with the 3 DoF Smart Template
- The 3DoF Smart Template uses Galil Controller to control the position of the 3 motors 

## Description
### Subscribers
- '/stage/initial_point', a PoseStamped with the initial position of the smart template

### Publishers
- '/stage/state/guide_pose', a PoseStamped with the current position of the template. It's refresh rate is given by the "timer_period"


### Action server

- The template node exposes the /move_stage action which takes a /stage_control_interfaces/action/MoveStage action message of the format:
  
*float64 x 
*float64 z 
*float64 eps 
*--- 
*float64 x 
*float64 z 
*float64 time 
*float64 error 
*int32 error_code 
*---
*float64 x
*float64 z
*float64 error
*float64 time


## Usage <a name="usage"></a>
### Network connection
- Setup the computer network to:
  * IP:
  * Subnet mask:
 
- 



