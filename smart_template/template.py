import gclib
import os
import rclpy
import numpy as np
import ament_index_python 
import serial
import time
import quaternion

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stage_control_interfaces.action import MoveStage
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt
from scipy.ndimage import median_filter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat
from std_msgs.msg import Int8

from datetime import datetime

MM_2_COUNT = 1088.9
COUNT_2_MM = 1.0/1088.9
SAFE_LIMIT = 60.0

class SmartTemplate(Node):

    def __init__(self):
        super().__init__('smart_template')      

        #Topics from sensor processing node
        self.subscription_initial_point = self.create_subscription(PoseStamped, '/stage/state/initial_point', self.initial_point_callback, 10)
        self.subscription_initial_point  # prevent unused variable warning

        #Published topics
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PoseStamped, 'stage/state/pose', 10)

        #Action server
        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

        #Start serial communication
        self.galil = gclib.py()
        self.galil.GOpen('192.168.0.99')
        #Initial position is always (0,0)
        self.galil.GCommand('DPA=0')
        self.galil.GCommand('DPB=0')
        self.galil.GCommand('PTA=1')
        self.galil.GCommand('PTB=1')

        #Stored values
        self.initial_point = np.empty(shape=[0,7])  # Initial point (at the begining of experiment)
        self.registration = np.empty(shape=[0,7])   # Registration transform (from aurora to stage)
        self.aurora = np.empty(shape=[0,7])         # All stored Aurora readings as they are sent

    def getMotorPosition(self):
        try:
            data_temp = self.galil.GCommand('TP')
            return data_temp
        except:
            return "error TP"

    # Timer to publish '/stage/state/pose'  
    def timer_stage_pose_callback(self):
        # Read guide position from robot motors
        read_position = self.getMotorPosition()
        Z = read_position.split(',')
        
        # Construct robot message to publish             
        # Add the initial point (home position)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        
        # Change self.initial_point is the initial position is not (0,0)
        msg.pose.position.x = 0.001*float(Z[0])*COUNT_2_MM# + self.initial_point[0,0]
        msg.pose.position.y = 0.0 
        # WARNING: Galil channel B inverted, that is why the my_goal is negative
        msg.pose.position.z = -0.001*float(Z[1])*COUNT_2_MM# + self.initial_point[2,0]
        #self.get_logger().info('motor read: %f %f ' % (float(Z[0]),float(Z[1])))
        msg.pose.orientation = Quaternion(w=float(1), x=float(0), y=float(0), z=float(0))
        self.publisher_stage_pose.publish(msg)
        self.get_logger().debug('stage_pose: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (msg.pose.position.x, msg.pose.position.y, \
                msg.pose.position.z,  msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.header.frame_id))

    # Initialization after needle is positioned in the initial point (after SPACE hit)
    def initial_point_callback(self, msg):
        if (self.initial_point.size == 0):
            self.ser.write(str.encode("DPA=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTA=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("DPB=0;"))
            time.sleep(0.02)
            self.ser.write(str.encode("PTB=1;"))
            time.sleep(0.02)
            self.ser.write(str.encode("SH;")) #Check this code
            self.AbsoluteMode = True
            self.get_logger().info('Needle guide at initial position')

            # Store initial point
            initial_point = msg.point
            self.initial_point = np.array([[initial_point.position.x, initial_point.position.y, initial_point.position.z, \
                                            1, 0, 0, 0]]).T

            # Load stored registration transform
            self.get_logger().info('Loading stored registration transform ...')
            try:
                self.registration = np.array(loadtxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration.csv'), delimiter=','))
            except IOError:
                self.get_logger().info('Could not find registration.csv file')
            self.get_logger().info('Registration = %s' %  (self.registration))

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
        if X > SAFE_LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = SAFE_LIMIT*MM_2_COUNT
        elif X < -SAFE_LIMIT*MM_2_COUNT:
            self.get_logger().info("Limit reach at axis %s" % (Channel))
            X = -SAFE_LIMIT*MM_2_COUNT
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
            
    # Execute a goal
    async def execute_callback(self, goal_handle):
        self.get_logger().debug('Executing goal...')
        feedback_msg = MoveStage.Feedback()
        # TODO: Feedback part
        feedback_msg.x = 0.0
        feedback_msg.z = 0.0

        # Start executing the action
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return MoveStage.Result()

        # Subtract initial point from goal because robot considers initial position to be (0,0)
        my_goal = goal_handle.request

        #########################################################
        # change self.initial_point is initial position is not (0,0)
        my_goal.x = my_goal.x #- self.initial_point[0,0]
        my_goal.z = my_goal.z #- self.initial_point[2,0]
        #########################################################

        self.get_logger().info("command %f %f" % (my_goal.x,my_goal.z))
        # Update control input - in Meters
        self.send_movement_in_counts(1000.0*my_goal.x*MM_2_COUNT,"A")
        
        # WARNING: Galil channel B inverted, that is why the my_goal is negative
        self.send_movement_in_counts(-1000.0*my_goal.z*MM_2_COUNT,"B")
      
        data = self.getMotorPosition()
        # TODO: populate feedback
        feedback_msg.x = float(0.0)

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()

        # Populate result message
        result = MoveStage.Result()
        result.x = feedback_msg.x
        self.get_logger().debug('Returning result: {0}'.format(result.x))

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
