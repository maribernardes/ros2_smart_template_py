import rclpy
import numpy as np
import time
import math

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import ControllerCommand, GetPoint

from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from scipy.ndimage import median_filter

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8

from datetime import datetime

HOME_X = 0.0
HOME_Y = 0.0
HOME_Z = 0.0

STEP_X = 0.05
STEP_Y = 0.1
STEP_Z = 0.05

SAFE_LIMIT = 60.0

TIMEOUT = 5             # timeout (sec) for move_stage action server 

#########################################################################
#
# Virtual Template
#
# Description:
# This node implements a virtual node to emulate the SmartTemplate
# Implements the robot service and action servers
#
# Subscribes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
#
# Action/service clients:
# '/move_stage' (smart_control_interfaces.action.MoveStage) - robot frame
# '/command'    (smart_control_interfaces.srv.ControllerCommand) - robot frame
# 
#########################################################################

class VirtualSmartTemplate(Node):

    def __init__(self):
        super().__init__('virtual_smart_template')      

#### Published topics ###################################################

        # Current position
        timer_period_stage = 0.3  # seconds
        self.timer_stage = self.create_timer(timer_period_stage, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PointStamped, '/stage/state/guide_pose', 10)

#### Action/Service server ##############################################

        self._action_server = ActionServer(self, MoveStage, '/stage/move', execute_callback=self.execute_move_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.move_callback, cancel_callback=self.cancel_move_callback)
        self.command_server = self.create_service(ControllerCommand, '/stage/command', self.command_callback, callback_group=ReentrantCallbackGroup())
        self.current_position_server = self.create_service(GetPoint, '/stage/get_position', self.current_position_callback, callback_group=ReentrantCallbackGroup())

#### Stored variables ###################################################

        self.position = np.empty(shape=[0,3])           # Current position
        self.abort = False                              # Flag to abort command

#### Node initialization ###################################################

        # Initial home position
        self.position = np.array([HOME_X, HOME_Y, HOME_Z])
        # Motion step
        self.motion_step = np.array([STEP_X, STEP_Y, STEP_Z])

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Publishing callbacks ###################################################

    # Timer to publish '/stage/state/pose'  
    def timer_stage_pose_callback(self):
        # Read guide position from robot motors
        position = self.get_position()
        # Construct robot message to publish             
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.point.x = position[0]
        msg.point.y = position[1]
        msg.point.z = position[2]
        self.publisher_stage_pose.publish(msg)

#### Service functions ###################################################

    # Return current robot position
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

    # Command robot
    def command_callback(self, request, response):
        command = request.command
        self.get_logger().debug('Received command request')
        self.get_logger().info('Command %s' %(command))
        eps = 0.001
        if command == 'HOME':
            goal = np.array([0.0, 0.0, 0.0])
            response.response = 'Command HOME sent'
            self.emulate_motion(goal, eps)
        elif command == 'RETRACT':
            position = self.get_position()
            goal = np.array([position[0], 0.0, position[2]])
            response.response = 'Command RETRACT sent'
            self.emulate_motion(goal, eps)
        elif command == 'ABORT':
            self.abort = True
        self.get_logger().info(response.response)
        return response
    
#### Action functions ###################################################

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This action server allows multiple goals in parallel
    def move_callback(self, goal_request):
        self.get_logger().debug('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_move_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # Execute a goal
    async def execute_move_callback(self, goal_handle):
        self.get_logger().info('Executing move_stage...')
        feedback = MoveStage.Feedback()
        result = MoveStage.Result()
        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return result
        # Get goal
        my_goal = goal_handle.request
        goal = np.array([my_goal.x, my_goal.y, my_goal.z])
        eps = my_goal.eps
        self.get_logger().debug('Command %s' %(my_goal))
        # Send control inputs
        goal[0] = self.check_limits(goal[0],'A')
        goal[2] = self.check_limits(goal[2],'B')
        goal[1]= self.check_limits(goal[1],'C')
        self.emulate_motion(goal, eps)
        # Feedback loop (while goal is not reached or not timeout)
        timer_on = False
        start_time = time.time()
        timeout_time = start_time
        position = self.get_position()
        while True:
            time.sleep(0.5)
            # Check if aborted
            if self.abort is True:
                goal_handle.abort()
                result.error_code = 2   # service for abort
                break
            # Check current position
            prev_position = position
            position = self.get_position()
            err = self.distance_positions(goal, position)
            # Update feedback message
            feedback.x = position[0]
            feedback.y = position[1]
            feedback.z = position[2]
            feedback.error = err
            feedback.time = time.time()-start_time
            goal_handle.publish_feedback(feedback)
            # Check if reached target
            if err <= eps:
                goal_handle.succeed()
                result.error_code = 0   # no error
                break
            # Check if moving
            else:
                motion = self.distance_positions(position, prev_position)
                if (timer_on is False) and (motion < eps):
                    timer_on = True         # Set timer
                    timeout_time = time.time() 
                elif (timer_on is True) and (motion >= eps):
                    timer_on = False        # Reset timer
                    timeout_time = time.time()
            # Check if timeout:
            if (timer_on is True) and ((time.time()-timeout_time) >= TIMEOUT):
                goal_handle.abort()
                result.error_code = 1   # timeout
                break
        # Set result message
        result.x = position[0]
        result.y = position[1]
        result.z = position[2]
        result.error = self.distance_positions(goal, position)
        result.time = time.time()-start_time
        self.get_logger().info('Finished move_stage. Result error code: %s' %result.error_code)
        return result

#### Internal functions ###################################################

    def check_limits(self, X, Channel):
        if Channel == "C":
            return X
        if X > SAFE_LIMIT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = SAFE_LIMIT
        elif X < -SAFE_LIMIT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -SAFE_LIMIT
        return X
    
    def get_position(self):
        return np.copy(self.position)
    
    def distance_positions(self, goal, position):        
        return math.sqrt((goal[0]-position[0])**2+(goal[1]-position[1])**2+(goal[2]-position[2])**2)

    # Emulate real robot motion
    def emulate_motion(self, goal, eps):
        self.abort = False
        self.get_logger().info('<ROBOT IS MOVING>')
        while (self.distance_positions(self.position, goal) > eps):
            if self.abort is True:
                break
            delta = goal-self.position
            step = np.minimum(np.absolute(delta), self.motion_step)
            self.position = self.position + np.multiply(np.sign(delta), step)
            time.sleep(0.01)
        self.get_logger().info('<ROBOT STOPPED>')
   
########################################################################

def main(args=None):

    rclpy.init(args=args)

    virtual_template = VirtualSmartTemplate()
    virtual_template.get_logger().info('VIRTUAL SmartTemplate ready')

    # rclpy.spin(virtual_template)

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(virtual_template, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
