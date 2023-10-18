import os
import rclpy
import numpy as np
import quaternion
import datetime


from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import Transform
from scipy.ndimage import median_filter

INSERTION_STEP = -5.0        # 5mm insertion step

class MRITrackingInterface(Node):

    def __init__(self):
        super().__init__('mri_tracking_interface')

        #Declare node parameters
        self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

#### Subscribed topics ###################################################

        #Topics from 3D Slicer interface (OpenIGTLink Bridge)
        self.subscription_bridge_transform = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.bridge_transform_callback, 10)
        self.subscription_bridge_transform # prevent unused variable warning

        #Topics from robot node (robot)
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

#### Published topics ###################################################

        # Experiment initial robot position (robot frame)
        timer_period_initialize = 1.0  # seconds
        self.timer_initialize = self.create_timer(timer_period_initialize, self.timer_initialize_callback)        
        self.publisher_initial_point = self.create_publisher(PointStamped, '/stage/initial_point', 10)

        # Tip (robot frame)
        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/needle/state/tip_pose', 10)  #(stage frame)


#### Stored variables ###################################################
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform
        self.tip = np.empty(shape=[0,3])            # Tracked tip position (robot frame)
        self.initial_point = np.empty(shape=[0,3])  # Needle guide position at begining of experiment (robot frame)

        # Flags
        self.initialize_insertion = False           # Flag to initialize the experiment (record initial guide position and following steps)

#### Interface initialization ###################################################

        # Initialize zFrameToRobot transform
        # Fixed relation from geometry of robot and zFrame attachment
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Get tracked tip pose and convert to robot frame
    def bridge_transform_callback(self, msg):
        name = msg.name      
        if name=="CurrentTrackedTip": # Needle tip sensor (name is adjusted in PlusServer .xml file)
            # Get aurora new reading
            tip_zFrame = np.array([[msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, \
                msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z]])
            self.tip = pose_transform(tip_zFrame, self.zFrameToRobot)

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        if (self.initialize_insertion is True):
            self.initial_point = np.array([robot.position.x, robot.position.y, robot.position.z])
            self.get_logger().debug('Initial stage position in (%f, %f, %f)' %(self.initial_point[0], self.initial_point[1], self.initial_point[2])) 
            self.initialize_insertion = False

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.stage.size == 0):
            self.get_logger().info('Wait. Robot is not publishing')
        elif (self.listen_keyboard == True) : # If listerning to keyboard
            if (self.initial_point.size == 0) and (msg.data == 32):  # SPACE: initialize stage initial point and initial depth
                self.initialize_insertion = True

#### Publishing callbacks ###################################################
            
    # Publishes initial point
    def timer_initialize_callback(self):
        # Publishes only after experiment started (stored initial point is available)
        if (self.initial_point.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            # Publish robot initial point (robot frame)
            msg.point = Point(x=self.initial_point[0], y=self.initial_point[1], z=self.initial_point[2])
            self.publisher_initial_point.publish(msg)

    # Publishes needle tip transformed to robot frame
    def timer_tip_callback (self):
        # Publish last needle pose in robot frame
        if (self.tip.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
            msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
            self.publisher_tip.publish(msg)
            
########################################################################

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

# Function: pose_inv_transform
# DO: Transform pose to new reference frame with inverse transform 
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_inv_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf.conj()*q_orig
    p_new = q_tf.conj()*(p_orig-p_tf)*q_tf
    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

def main(args=None):
    rclpy.init(args=args)

    system_interface = MRITrackingInterface()
    system_interface.get_logger().info('Waiting for stage...')

    # Wait for stage and sensor to start publishing
    while rclpy.ok():
        rclpy.spin_once(system_interface)
        if(system_interface.stage.size == 0):
            pass
        else:
            system_interface.get_logger().info('Stage connected. Now place the needle at the Entry Point')
            system_interface.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            system_interface.get_logger().info('**** To initialize experiment, place needle at initial position and hit SPACE ****')
            system_interface.listen_keyboard = True
            break

    # Initialize insertion
    while rclpy.ok():
        rclpy.spin_once(system_interface)
        if system_interface.initial_point.size == 0: # Not initialized yet
            pass
        else:
            system_interface.get_logger().info('*****START NEEDLE INSERTION*****')
            break

    rclpy.spin(system_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    system_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()