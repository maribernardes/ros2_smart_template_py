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

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState

from datetime import datetime

# Horizontal
MM_2_COUNT_X = 715.0  
COUNT_2_MM_X = 0.0014
# Depth
MM_2_COUNT_Y = -2000.0/1.27
COUNT_2_MM_Y = -0.0005*1.27
# Vertical
MM_2_COUNT_Z = 1430.0 
COUNT_2_MM_Z = 0.0007

SAFE_LIMIT = 60.0

ERROR_GAIN = 500 #change the error from mm to counts

TIMEOUT = 30.0             # timeout (sec) for move_and_observe action server 


#########################################################################
#
# Template
#
# Description:
# This node runs the SmartTemplate communication with Galil and implement
# the robot service and action servers
#
# Publishes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
# '/joint_states'     (geometry_msgs.msg.PointStamped)  - robot frame
#
# Action/service clients:
# '/stage/move_and_observe'     (smart_template_interfaces.action.MoveAndObserve) - robot frame
# '/stage/move'                 (smart_template_interfaces.srv.Move) - robot frame
# '/stage/command'              (smart_template_interfaces.srv.Command) - robot frame
# '/stage/get_position'         (smart_template_interfaces.srv.GetPoint) - robot frame
# 
#########################################################################

#TODO: Set Limits from robot description


class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      

    #### Published topics ###################################################

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PointStamped, '/stage/state/guide_pose', 10)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

    #### Action/Service server ##############################################
       
        self._action_server = ActionServer(self, MoveAndObserve, '/stage/move_and_observe', execute_callback=self.execute_move_and_observe_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.move_and_observe_callback, cancel_callback=self.cancel_move_and_observe_callback)
        self.command_server = self.create_service(Command, '/stage/command', self.command_callback, callback_group=ReentrantCallbackGroup())
        self.move_server = self.create_service(Move, '/stage/move', self.move_callback, callback_group=ReentrantCallbackGroup())
        self.current_position_server = self.create_service(GetPoint, '/stage/get_position', self.current_position_callback, callback_group=ReentrantCallbackGroup())

    #### Connection initialization ###################################################

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
        # Set joint names
        self.joint_names = ['horizontal_joint', 'insertion_joint', 'vertical_joint']

#### Internal functions ###################################################

    # Get current robot position error
    def tell_error(self):
        try:
            data_temp = self.galil.GCommand('TE')
            data = data_temp.split(',')
            x = int(data[0])
            y = int(data[2])
            z = int(data[1])
            self.get_logger().info('errX=%f, errY=%f, errZ=%f' %(x, y, z))
            return [x, y, z]
        except:
            return "error TE"

    # Get current robot position
    def get_position(self):
        try:
            data_temp = self.galil.GCommand('TP')
            data = data_temp.split(',')
            # Change self.initial_point is the initial position is not (0,0)
            # WARNING: Galil channel B inverted, that is why the value is negative (NOT SURE IF STILL TRUE)
            # TODO: Check the count ratio for each motor. Check channel B direction
            x = float(data[0])*COUNT_2_MM_X# + self.initial_point[0,0] # CHANNEL A
            y = float(data[2])*COUNT_2_MM_Y# + self.initial_point[1,0] # CHANNEL C
            z = float(data[1])*COUNT_2_MM_Z# + self.initial_point[2,0] # CHANNEL B
            #self.get_logger().info('x=%f, y=%f, z=%f' %(x, y, z))
            return [x, y, z]
        except:
            return "error TP"

    # Abort any ongoing motion (stop where it is)
    # TODO: Check best implementation for this using Galil
    def abort_motion(self):
        # self.abort = True
        self.galil.GCommand('SH')
        self.get_logger().info('ABORT')

    # Send desired movement to each channel
    # WARNING: Galil channel B inverted, that is why the my_goal is negative
    # TODO: Check if Channel B is still inverted
    def send_movement(self, goal):
        self.send_movement_in_counts(goal[0]*MM_2_COUNT_X,"A")   #X = CH_A
        self.send_movement_in_counts(goal[2]*MM_2_COUNT_Z,"B")   #Z = CH_B
        self.send_movement_in_counts(goal[1]*MM_2_COUNT_Y,"C")   #Y = CH_C

    # Check if X counts exceeds channel limit
    def check_limits(self, X, Channel):
        if Channel == "C":
            return X
        if X > SAFE_LIMIT*MM_2_COUNT_X:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = SAFE_LIMIT*MM_2_COUNT_X
        elif X < -SAFE_LIMIT*MM_2_COUNT_X:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -SAFE_LIMIT*MM_2_COUNT_X
        return X

    # Send X movement counts to channel
    def send_movement_in_counts(self, X, Channel):
        try:
            X = self.check_limits(X, Channel)
            send = "PA%s=%d" % (Channel, int(X))
            self.galil.GCommand(send)
            send = "PA%s=%d" % (Channel, int(X))
            self.get_logger().info("Sent to Galil PA%s=%d" % (Channel,X))
            return True
        except:
            return False
    
    # Return 3D euclidean distance between two positions
    def distance_positions(self, goal, position):        
        return math.sqrt((goal[0]-position[0])**2+(goal[1]-position[1])**2+(goal[2]-position[2])**2)

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
            self.send_movement(goal)
            response.response = 'Command HOME sent'
        elif command == 'RETRACT':
            position = self.get_position()
            goal = np.array([position[0], 0.0, position[2]])
            self.send_movement(goal)
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
            # Simulate robot movement logic here
            # e.g., call hardware control interfaces
            goal = np.array([request.x, request.y, request.z])
            goal[0] = self.check_limits(goal[0],'A')
            goal[2] = self.check_limits(goal[2],'B')
            goal[1]= self.check_limits(goal[1],'C')
            self.send_movement(goal)
            response.response = "Success: Robot moved to the specified position."
        except Exception as e:
            response.response = f"Error: {str(e)}"
        return response


#### Publishing callbacks ###################################################

    # Publishes current robot pose
    def timer_stage_pose_callback(self):
        # Read guide position from robot motors
        position = self.get_position()
        # Construct robot message to publish             
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'stage'
        msg.point.x = position[0]
        msg.point.y = position[1]
        msg.point.z = position[2]
        self.publisher_stage_pose.publish(msg)
        self.get_logger().debug('stage_pose: x=%f, y=%f, z=%f in %s frame'  % (msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id))
        # Update joint_state message to publish
        joint_state_msg = JointState()                
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [0.001*position[0], 0.001*position[1], 0.001*position[2]] # Convert from mm to m
        self.publisher_joint_states.publish(joint_state_msg)

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
        self.get_logger().debug('Executing move_stage...')
        feedback = MoveAndObserve.Feedback()
        result = MoveAndObserve.Result()
        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return result
        # Get goal
        my_goal = goal_handle.request
        goal = [my_goal.x, my_goal.y, my_goal.z]
        self.get_logger().info("Command %s" % (goal))
        # Send control inputs
        self.send_movement(goal)
        # # Feedback loop (while goal is not reached or not timeout)
        # timer_on = False
        start_time = time.time()
        result.error_code = 0

        while True:
            time.sleep(0.1)
            # Check current position error
            err = self.tell_error()

        #    TODO or not TODO # Update feedback message

            if self.abort == True:
                # self.galil.GCommand('SH')
                goal_handle.abort()
                result.error_code = 2   # abort
                # self.abort = False
                self.get_logger().info('Chegou no abort')
                break

           # Check if reached target
            if (abs(err[0]) <= my_goal.eps*ERROR_GAIN) and (abs(err[1]) <= my_goal.eps*ERROR_GAIN) and (abs(err[2]) <= my_goal.eps*ERROR_GAIN):
                goal_handle.succeed()
                result.error_code = 0
                break

            if (time.time()-start_time) >= TIMEOUT:
                goal_handle.abort()
                result.error_code = 1   # timeout
                break


        position = self.get_position()
        # Set result message
        result.x = position[0]
        result.y = position[1]
        result.z = position[2]
        result.error = self.distance_positions(goal, position)
        result.time = time.time()-start_time
        self.get_logger().info('Finished move_stage')
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