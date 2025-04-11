import gclib
import os
import rclpy
import numpy as np
import ament_index_python 
import serial
import time
import math
import quaternion

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_template_interfaces.action import MoveAndObserve
from smart_template_interfaces.srv import Command, Move, GetPoint
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from scipy.ndimage import median_filter

from geometry_msgs.msg import PointStamped, Point
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState

from datetime import datetime
import xml.etree.ElementTree as ET
from collections import OrderedDict

TIMEOUT = 20.0      # timeout (sec) for move_and_observe action server 

#########################################################################
#
# Template
#
# Description:
# This node runs the SmartTemplate communication with Galil and implement
# the robot service and action servers
#
# Publishes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - [mm] robot frame
# '/joint_states'               (sensor_msgs.msg.JointState)      - [m] robot frame
#
# Subscribe:
# '/desired_position'           (geometry_msgs.msg.Point)  - [mm] robot frame
#
# Action/service clients:
# '/stage/move_and_observe'     (smart_template_interfaces.action.MoveAndObserve) - robot frame
# '/stage/move'                 (smart_template_interfaces.srv.Move) - robot frame
# '/stage/command'              (smart_template_interfaces.srv.Command) - robot frame
# '/stage/get_position'         (smart_template_interfaces.srv.GetPoint) - robot frame
# 
#########################################################################

class JointInfo:
    def __init__(self):
        self.names = []
        self.channels = []
        self.limits_lower = []
        self.limits_upper = []
        self.mm_to_count = []
        self.count_to_mm = []

    # Add a new joint to the list
    def add(self, name, channel, lower, upper, mm_to_count, count_to_mm):
        self.names.append(name)
        self.channels.append(channel)
        self.limits_lower.append(lower)
        self.limits_upper.append(upper)
        self.mm_to_count.append(mm_to_count)
        self.count_to_mm.append(count_to_mm)

    def index(self, joint_name: str) -> int:
        return self.names.index(joint_name)
    
class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      
        np.set_printoptions(precision=4, suppress=False, floatmode='maxprec_equal', linewidth=120)

    #### Get joint limits from robot description ###################################################
        self.declare_parameter('robot_description', '')
        urdf_str = self.get_parameter('robot_description').get_parameter_value().string_value
        self.joints = self.parse_joint_info_from_urdf(urdf_str)
        if self.joints is None:
            self.get_logger().fatal("Joint configuration could not be loaded from URDF.")
            return 

    #### Published topics ##################################################
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PointStamped, '/stage/state/guide_pose', 10)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

    #### Subscribed topics ##################################################
        self.subscription_desired_position = self.create_subscription(Point, '/desired_position', self.desired_position_callback, 10)
        self.subscription_desired_position # prevent unused variable warning
        
    #### Action/Service server ##############################################
       
        self._action_server = ActionServer(self, MoveAndObserve, '/stage/move_and_observe', execute_callback=self.execute_move_and_observe_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.move_and_observe_callback, cancel_callback=self.cancel_move_and_observe_callback)
        self.command_server = self.create_service(Command, '/stage/command', self.command_callback, callback_group=ReentrantCallbackGroup())
        self.move_server = self.create_service(Move, '/stage/move', self.move_callback, callback_group=ReentrantCallbackGroup())
        self.current_position_server = self.create_service(GetPoint, '/stage/get_position', self.current_position_callback, callback_group=ReentrantCallbackGroup())

    #### Connection initialization #########################################

        # Start serial communication
        self.galil = gclib.py()
        self.galil.GOpen('192.168.0.99')
        # Initial position is always set to (0,0,0)
        self.galil.GCommand('DPA=0')
        self.galil.GCommand('DPB=0')
        self.galil.GCommand('DPC=0')
        self.galil.GCommand('PTA=1')
        self.galil.GCommand('PTB=1')
        self.galil.GCommand('PTC=1')
        self.AbsoluteMode = True
        self.abort = False

#### Kinematic model ###################################################

    # Forward kinematics
    def fk_model(self, joints: np.ndarray) -> np.ndarray:
        x = joints[0]    # horizontal_joint  (channel A)
        z = joints[1]    # vertical_joint    (channel B)
        y = joints[2]    # insertion_joint   (channel C) 
        return np.array([x, y, z], dtype=float)

    # Inverse kinematics
    def ik_model(self, position: np.ndarray) -> np.ndarray:
        horizontal_joint = position[0]  # horizontal_joint  (channel A)
        vertical_joint = position [2]   # vertical_joint    (channel B)
        insertion_joint = position[1]   # insertion_joint   (channel C) 
        return np.array([horizontal_joint, vertical_joint, insertion_joint], dtype=float)

#### Internal functions ###################################################
    
    # Load joint information from URDF
    def parse_joint_info_from_urdf(self, urdf_str: str) -> JointInfo:
        raw_channels = {}
        raw_limits = {}
        raw_mm_to_count = {}
        joint_info = JointInfo()
        try:
            root = ET.fromstring(urdf_str)
            for joint_elem in root.findall('joint'):
                name = joint_elem.attrib.get('name')
                if not name:
                    continue
                # Channel
                channel_elem = joint_elem.find('channel')
                if channel_elem is not None:
                    raw_channels[name] = channel_elem.text.strip()
                # Limits
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    try:
                        lower = float(limit_elem.attrib.get('lower', 'nan')) * 1000
                        upper = float(limit_elem.attrib.get('upper', 'nan')) * 1000
                        raw_limits[name] = (lower, upper)
                    except ValueError:
                        self.get_logger().warn(f"Invalid limit values for joint '{name}'")
                # mm_to_count
                mm_elem = joint_elem.find('mm_to_count')
                if mm_elem is not None:
                    try:
                        mm = float(mm_elem.text.strip())
                        raw_mm_to_count[name] = mm
                    except ValueError:
                        self.get_logger().warn(f"Invalid mm_to_count value for joint '{name}'")
        except ET.ParseError as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return None
        # Sort by channel order (A, B, C, ...)
        sorted_joints = sorted(raw_channels.items(), key=lambda item: item[1])
        for name, channel in sorted_joints:
            lower, upper = raw_limits.get(name, (float('-inf'), float('inf')))
            mm_to_count = raw_mm_to_count.get(name, 1.0)
            count_to_mm = 1.0 / mm_to_count if mm_to_count != 0.0 else 0.0
            joint_info.add(name, channel, lower, upper, mm_to_count, count_to_mm)
            self.get_logger().info(f"{name}: channel={channel}, limits=({lower:.2f}, {upper:.2f}) mm, "
                        f"mm_to_count={mm_to_count}, count_to_mm={count_to_mm}")
        return joint_info

    # Get current robot joint values [mm]
    def get_joints(self) -> np.ndarray:
        try:
            data_temp = self.galil.GCommand('TP')
            data = data_temp.split(',')[:len(self.joints.names)]
            counts = np.fromiter(map(int, data), dtype=float)
            return (counts*np.asarray(self.joints.count_to_mm, dtype=float))
        except Exception as e:
            self.get_logger().error(f"Failed to get joint values: {e}")
            return None
        
    # Get current joints error [mm]
    def get_joints_err(self) -> np.ndarray:
        try:
            data_temp = self.galil.GCommand('TE')
            data = data_temp.split(',')[:len(self.joints.names)]
            err_count = np.fromiter(map(int, data), dtype=float)
            err_joints = err_count * np.asarray(self.joints.count_to_mm, dtype=float)
            self.get_logger().info(f'Joint errors [mm]: {err_joints}')
            return err_joints
        except Exception as e:
            self.get_logger().error(f"Failed to get joint errors: {e}")
            return None

    # Get current robot position
    def get_position(self) -> np.ndarray:
        joints = self.get_joints()
        if joints is None:
            return None
        return self.fk_model(joints)
                        
    # Get current robot position error
    def get_position_error(self) -> np.ndarray:
        err_joints = self.get_joints_err()
        if err_joints is None:
            return None
        err_position = self.fk_model(err_joints)
        self.get_logger().info(f'Error (joint/mm): {err_joints}, FK error: {err_position}')
        return err_position

    # Calculate euclidean error
    def error_3d(self, err: np.ndarray) -> float:
        return float(np.linalg.norm(err))
    
    # TODO: Check best implementation for this using Galil
    # Abort any ongoing motion (stop where it is)
    def abort_motion(self):
        self.abort = True
        self.galil.GCommand('SH')
        self.get_logger().info('ABORT')

    # Checks each joint value to be withing the joint's defined limits
    # The order of joint_values must match self.joints.names
    def check_limits(self, joint_values: list[float]) -> list[float]:
        if len(joint_values) != len(self.joints.names):
            raise ValueError("Length of joint_values does not match number of joints")
        final_values = []
        for i, value in enumerate(joint_values):
            lower = self.joints.limits_lower[i]
            upper = self.joints.limits_upper[i]
            if value < lower or value > upper:
                self.get_logger().warn(
                    f"{self.joints.names[i]} value {value:.2f} mm out of bounds "
                    f"[{lower:.2f}, {upper:.2f}] — clipping to limit."
                )
            capped = max(lower, min(value, upper))
            final_values.append(capped)
        return final_values

    # Sends a movement command to all joints based on the goal [x, y, z] in mm.
    def position_control(self, goal: np.ndarray):
        joints_mm = self.ik_model(goal)                           
        mm_to_count = np.array(self.joints.mm_to_count, dtype=float)
        joints_count = np.round(joints_mm * mm_to_count).astype(int)
        # Send values to each joint/channel
        for channel, count_value in zip(self.joints.channels, joints_count):
            try:
                send = f"PA{channel}={count_value}"
                self.galil.GCommand(send)
                self.get_logger().info(f"Sent to Galil: {send}")
            except Exception as e:
                self.get_logger().error(f"Failed to send to channel {channel}: {e}")

#### Listening callbacks ###################################################

    # A request for desired position was sent
    def desired_position_callback(self, msg):
        goal = np.array([msg.x, msg.y, msg.z])
        #goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.get_logger().info(f'Received request: x={goal[0]}, y={goal[1]}, z={goal[2]}')
        self.position_control(goal)

#### Publishing callbacks ###################################################

    # Publishes current robot pose
    def timer_stage_pose_callback(self):
        # Read joint positions from robot encoders
        joints_mm = self.get_joints()
        if joints_mm is not None:
            position = self.fk_model(joints_mm)
            # Construct robot message to publish             
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.point.x = position[0]
            msg.point.y = position[1]
            msg.point.z = position[2]
            self.publisher_stage_pose.publish(msg)
            self.get_logger().debug('stage_pose [mm]: x=%f, y=%f, z=%f in %s frame'  % (msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id))
            # Update joint_state message to publish
            joint_state_msg = JointState()                
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joints.names
            joint_state_msg.position = [0.001*joints_mm[0], 0.001*joints_mm[1], 0.001*joints_mm[2]] # Convert from mm to m (ROS2 tools expect meters)
            self.publisher_joint_states.publish(joint_state_msg)

#### Service functions ###################################################

    # Current position service request
    def current_position_callback(self, request, response):
        self.get_logger().debug('Received current position request')
        try:
            position = self.get_position()
            response.valid = True
            response.x = position[0]
            response.y = position[1]
            response.z = position[2]
        except:
            response.valid = False
        return response
    
    # Command service request
    def command_callback(self, request, response):
        command = request.command
        self.get_logger().debug('Received command request')
        self.get_logger().info('Command %s' %(command))
        if command == 'HOME':
            goal = np.array([0.0, 0.0, 0.0])
            self.position_control(goal)
            response.response = 'Command HOME sent'
        elif command == 'RETRACT':
            position = self.get_position()
            goal = np.array([position[0], 0.0, position[2]])
            self.position_control(goal)
            response.response = 'Command RETRACT sent'
        elif command == 'ABORT':
            self.abort_motion()
        return response

    # Move robot
    def move_callback(self, request, response):
        # Log the incoming request
        self.get_logger().info(f'Received request: x={request.x}, y={request.y}, z={request.z}, eps={request.eps}')
        try:
            if request.eps < 0.0:
                raise ValueError("Epsilon cannot be negative")
            goal = np.array([request.x, request.y, request.z])
            self.position_control(goal)
            response.response = "Success: Robot moved to the specified position."
        except Exception as e:
            response.response = f"Error: {str(e)}"
        return response

#### Action functions ###################################################

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def move_and_observe_callback(self, goal_request):
        self.get_logger().debug('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_move_and_observe_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    # Execute a goal
    async def execute_move_and_observe_callback(self, goal_handle):
        self.get_logger().debug('Executing move_and_observe...')
        feedback = MoveAndObserve.Feedback()
        result = MoveAndObserve.Result()
        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return result
        # Get goal and send
        my_goal = goal_handle.request
        goal = [my_goal.x, my_goal.y, my_goal.z]
        self.get_logger().info(' Move to %s' %(goal))
        self.position_control(goal) 
        ####### Feedback loop (while goal is not reached or not timeout)
        # TODO or not TODO: Update feedback message
        start_time = time.time()
        result.error_code = 0
        while True:
            time.sleep(0.1)
            if self.abort == True:
                goal_handle.abort()
                result.error_code = 2   # abort
                self.abort = False
                break
            # Check if reached target
            joints_err = self.get_joints_err()
            if all(abs(e) < my_goal.eps for e in joints_err):
                goal_handle.succeed()
                result.error_code = 0
                break
            if (time.time()-start_time) >= TIMEOUT:
                goal_handle.abort()
                result.error_code = 1   # timeout
                break
        # Set result message
        position = self.get_position()
        result.x = position[0]
        result.y = position[1]
        result.z = position[2]
        result.error = self.error_3d(position-goal)
        result.time = time.time()-start_time
        self.get_logger().info('Finished move_and_observe')
        return result

########################################################################

def main():
    rclpy.init()
    smart_template = SmartTemplate()
    smart_template.get_logger().info('SmartTemplate ready')
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(smart_template, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    smart_template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()