import os
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray, String

#########################################################################
#
# Initialization Node
#
# Description:
# This node monitors keyboard inputs for a first SPACE hit that signals
# the initial robot position and begining of the insertion
#
# Subscribes:   
# '/keyboard/key'           (std_msgs.msg.Int8)
# '/stage/state/guide_pose' (geometry_msgs.msg.PoseStamped)  - robot frame
#
# Publishes:    
# '/stage/initial_point'    (geometry_msgs.msg.PoseStamped) - robot frame
#
#########################################################################

class Initialization(Node):

    def __init__(self):
        super().__init__('initialization')

#### Subscribed topics ###################################################

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

#### Published topics ###################################################

        # Experiment initial robot position (robot frame)
        timer_period_initialize = 3.0  # seconds
        self.timer_initialize = self.create_timer(timer_period_initialize, self.timer_initialize_callback)        
        self.publisher_initial_point = self.create_publisher(PoseStamped, '/stage/initial_point', 10)


#### Stored variables ###################################################
        
        self.initial_point = np.empty(shape=[0,7])  # Stage position at begining of experiment
        self.stage = np.empty(shape=[0,7])          # Stage positions: horizontal / depth / vertical 
        self.listen_keyboard = False                # Flag for waiting keyboard input (set robot initial position)

#### Node initialization ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z, 
                               robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.stage.size == 0):
            self.get_logger().info('Wait. Robot is not publishing')
        elif (self.listen_keyboard == True) : # If listerning to keyboard
            if (self.initial_point.size == 0) and (msg.data == 32):  # SPACE: initialize stage initial point and initial depth
                # Initialize robot
                self.initial_point = self.stage
                self.get_logger().debug('Initial robot position: %s' %(self.initial_point)) 
                # Publishes immediately
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.pose.position = Point(x=self.initial_point[0], y=self.initial_point[1], z=self.initial_point[2])
                msg.pose.orientation = Quaternion(w=self.initial_point[3], x=self.initial_point[4], y=self.initial_point[5], z=self.initial_point[6])
                self.publisher_initial_point.publish(msg)

#### Publishing callbacks ###################################################
            
    # Publishes initial point
    def timer_initialize_callback(self):
        # Publishes only after experiment started (stored initial point is available)
        if (self.initial_point.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            # Publish robot initial point (robot frame)
            msg.pose.position = Point(x=self.initial_point[0], y=self.initial_point[1], z=self.initial_point[2])
            msg.pose.orientation = Quaternion(w=self.initial_point[3], x=self.initial_point[4], y=self.initial_point[5], z=self.initial_point[6])
            self.publisher_initial_point.publish(msg)

########################################################################

def main(args=None):
    rclpy.init(args=args)

    initialization = Initialization()
    initialization.get_logger().info('Waiting for robot...')

    # Wait for stage and sensor to start publishing
    while rclpy.ok():
        rclpy.spin_once(initialization)
        if(initialization.stage.size == 0): # Robot has not published yet (wait for it to connect)
            pass
        else:
            initialization.get_logger().info('Robot is connected. ')
            initialization.get_logger().info('**** To initialize experiment, place needle at initial position and hit SPACE ****')
            initialization.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            initialization.listen_keyboard = True
            break

    # Initialize insertion
    while rclpy.ok():
        rclpy.spin_once(initialization)
        if initialization.initial_point.size == 0: # Not initialized yet
            pass
        else:
            initialization.get_logger().info('***** Initialization Sucessfull *****')
            break

    rclpy.spin(initialization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    initialization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()