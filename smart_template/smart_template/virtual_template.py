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
from smart_control_interfaces.action import MoveStage
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from scipy.ndimage import median_filter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8

from datetime import datetime

HOME_X = 1.0
HOME_Y = 1.0
HOME_Z = 1.0

STEP_X = 0.05
STEP_Y = 0.1
STEP_Z = 0.05

SAFE_LIMIT = 60.0

TIMEOUT = 5             # timeout (sec) for move_stage action server 

class VirtualSmartTemplate(Node):

    def __init__(self):
        super().__init__('virtual_smart_template')      

#### Subscribed topics ###################################################

        # # Topic from initialization node
        # self.subscription_initial_point = self.create_subscription(PoseStamped, '/stage/initial_point', self.initial_point_callback, 10)
        # self.subscription_initial_point  # prevent unused variable warning

#### Published topics ###################################################

        # Current position
        timer_period_stage = 0.3  # seconds
        self.timer_stage = self.create_timer(timer_period_stage, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PoseStamped, '/stage/state/guide_pose', 10)

#### Action server ###################################################

        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

#### Stored variables ###################################################

        self.initial_point = np.empty(shape=[0,3])      # Initial point (at the begining of experiment)
        self.position = np.empty(shape=[0,3])           # Current position

#### Node initialization ###################################################

        # Initial home position
        self.position = np.array([HOME_X, HOME_Y, HOME_Z])
        # Motion step
        self.motion_step = np.array([STEP_X, STEP_Y, STEP_Z])

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # # Initialization after needle is positioned in the initial point (after SPACE hit)
    # def initial_point_callback(self, msg):
    #     if (self.initial_point.size == 0):  # Do only once
    #         # Store initial point
    #         initial_point = msg.pose
    #         self.initial_point = np.array([initial_point.position.x, initial_point.position.y, initial_point.position.z])

#### Publishing callbacks ###################################################

    # Timer to publish '/stage/state/pose'  
    def timer_stage_pose_callback(self):
        # Read guide position from robot motors
        position = self.get_position()
        # Construct robot message to publish             
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.publisher_stage_pose.publish(msg)


#### Internal functions ###################################################

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def goal_callback(self, goal_request):
        self.get_logger().debug('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

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
        while (self.distance_positions(self.position, goal) > eps):
            delta = goal-self.position
            step = np.minimum(np.absolute(delta), self.motion_step)
            self.position = self.position + np.multiply(np.sign(delta), step)
            time.sleep(0.01)
   
    # Execute a goal
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
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
        #########################################################

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
                result.error_code = 0
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
        return result

########################################################################

def main(args=None):

    rclpy.init(args=args)

    virtual_template = VirtualSmartTemplate()

    rclpy.spin(virtual_template)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
