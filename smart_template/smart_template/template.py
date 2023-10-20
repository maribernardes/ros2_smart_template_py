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

MM_2_COUNT_X = 715.0
COUNT_2_MM_X = 0.0014
MM_2_COUNT_Y = 2000.0
COUNT_2_MM_Y = 0.0005
MM_2_COUNT_Z = 1430.0
COUNT_2_MM_Z = 0.0007


SAFE_LIMIT = 60.0

TIMEOUT = 2             # timeout (sec) for move_stage action server 

class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      

        #Topics from sensor processing node
        self.subscription_initial_point = self.create_subscription(PoseStamped, '/stage/initial_point', self.initial_point_callback, 10)
        self.subscription_initial_point  # prevent unused variable warning

        #Published topics
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PoseStamped, '/stage/state/guide_pose', 10)

        #Action server
        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

        #Start serial communication
        self.galil = gclib.py()
        self.galil.GOpen('192.168.0.99')
        #Initial position is always (0,0)
        self.galil.GCommand('DPA=0')
        self.galil.GCommand('DPB=0')
        self.galil.GCommand('DPC=0')
        self.galil.GCommand('PTA=1')
        self.galil.GCommand('PTB=1')
        self.galil.GCommand('PTC=1')

        #Stored values
        self.initial_point = np.empty(shape=[0,3])  # Initial point (at the begining of experiment)

    def get_position(self):
        try:
            data_temp = self.galil.GCommand('TP')
            data = data_temp.split(',')
            # Change self.initial_point is the initial position is not (0,0)
            # WARNING: Galil channel B inverted, that is why the value is negative (NOT SURE IF STILL TRUE)
            # TODO: Check the count ratio for each motor. Check channel B direction
            x = float(data[0])*COUNT_2_MM_X# + self.initial_point[0,0] # CHANNEL A
            y = float(data[2])*COUNT_2_MM_Y# + self.initial_point[1,0] # CHANNEL C
            z =-float(data[1])*COUNT_2_MM_Z# + self.initial_point[2,0] # CHANNEL B
            # self.get_logger().info('x=%f, y=%f, z=%f' %(x, y, z))
            return [x, y, z]
        except:
            return "error TP"

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
        msg.pose.orientation = Quaternion(w=float(1), x=float(0), y=float(0), z=float(0))
        self.publisher_stage_pose.publish(msg)
        self.get_logger().debug('stage_pose: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (msg.pose.position.x, msg.pose.position.y, \
                msg.pose.position.z,  msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.header.frame_id))

    # Initialization after needle is positioned in the initial point (after SPACE hit)
    def initial_point_callback(self, msg):
        if (self.initial_point.size == 0):  # Do only once
            self.ser.write(str.encode("DPA=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTA=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("DPB=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTB=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("DPC=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTC=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("SH;")) #Check this code
            self.AbsoluteMode = True
            self.get_logger().info('Needle guide at initial position')

            # Store initial point
            initial_point = msg.point
            self.initial_point = np.array([initial_point.position.x, initial_point.position.y, initial_point.position.z])

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

    def check_limits(self,X,Channel):
        if X > SAFE_LIMIT*MM_2_COUNT_X:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = SAFE_LIMIT*MM_2_COUNT_X
        elif X < -SAFE_LIMIT*MM_2_COUNT_X:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -SAFE_LIMIT*MM_2_COUNT_X
        return X

    def send_movement_in_counts(self,X,Channel):
        try:
            X = self.check_limits(X,Channel)
            send = "PA%s=%d" % (Channel,int(X))
            self.galil.GCommand(send)
            send = "PA%s=%d" % (Channel,int(X))
            self.get_logger().info("Sent to Galil PA%s=%d" % (Channel,X))
            return True
        except:
            return False
    
    def distance_positions(self, goal, position):        
        return math.sqrt((goal[0]-position[0])**2+(goal[1]-position[1])**2+(goal[2]-position[2])**2)
    
    # Execute a goal
    async def execute_callback(self, goal_handle):
        self.get_logger().debug('Executing goal...')
        feedback = MoveStage.Feedback()
        result = MoveStage.Result()

        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return result

        # Get goal
        my_goal = goal_handle.request
        #########################################################
        # Subtract initial point from goal because robot considers initial position to be (0,0)
        # change self.initial_point if initial position is not (0,0)
        goal = [my_goal.x, my_goal.y, my_goal.z]
        # goal = [my_goal.x+initial_position[0], my_goal.y+initial_position[1], my_goal.z+initial_position[2]]
        self.get_logger().info("Command %s" % (goal))
        #########################################################

        # Send control inputs
        # WARNING: Galil channel B inverted, that is why the my_goal is negative
        # TODO: Check if Channel B is still inverted
        self.send_movement_in_counts(goal[0]*MM_2_COUNT_X,"A")    #X = CH_A
        self.send_movement_in_counts(-goal[2]*MM_2_COUNT_Z,"B")   #Z = CH_B
        self.send_movement_in_counts(goal[1]*MM_2_COUNT_Y,"C")    #Y = CH_C
        
        # # Feedback loop (while goal is not reached or not timeout)
        # timer_on = False
        start_time = time.time()
        # timeout_time = start_time
        # position = self.get_position()
        # while True:
        #     time.sleep(0.2)
        #     # Check current position
        #     prev_position = position
        #     position = self.get_position()
        #     err = self.distance_positions(goal, position)
        #     # Update feedback message
        #     feedback.x = position[0]
        #     feedback.y = position[1]
        #     feedback.z = position[2]
        #     feedback.error = err
        #     feedback.time = time.time()-start_time
        #     goal_handle.publish_feedback(feedback)
        #     # Check if reached target
        #     if err <= my_goal.eps:
        #         goal_handle.succeed()
        #         result.error_code = 0
        #         break
        #     # Check if moving
        #     else:
        #         motion = self.distance_positions(position, prev_position)
        #         if (timer_on is False) and (motion < my_goal.eps):
        #             timer_on = True         # Set timer
        #             timeout_time = time.time() 
        #         elif (timer_on is True) and (motion >= my_goal.eps):
        #             timer_on = False        # Reset timer
        #             timeout_time = time.time()
        #     # Check if timeout:
        #     if (timer_on is True) and ((time.time()-timeout_time) >= TIMEOUT):
        #         goal_handle.abort()
        #         result.error_code = 1   # timeout
        #         break

        # Make always success (Temporary)
        goal_handle.succeed()
        position = self.get_position()
        result.error_code = 0

        # Set result message
        result.x = position[0]
        result.y = position[1]
        result.z = position[2]
        result.error = self.distance_positions(goal, position)
        result.time = time.time()-start_time
        return result

########################################################################

def main():
    rclpy.init()
    smart_template = SmartTemplate()
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
