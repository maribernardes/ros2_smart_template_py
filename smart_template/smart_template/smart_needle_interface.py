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
# SmartNeedle Interface Node
#
# Description:
# This node gets current needle shape, extracts the tip pose and converts
# it to robot frame. It also gets robot current position and extracts the
# needle base pose in needle frame.
# If use_slicer is set to True, it also pushes the needle shape to 3DSlicer
# throught OpenIGTLink bridge in zFrame coordinates
#
# Subscribes:   
# '/needle/state/current_shape' (geometry_msgs.msg.PoseArray)    - needle frame
# '/stage/state/pose'           (geometry_msgs.msg.PoseStamped)  - robot frame
# '/stage/initial_point'        (geometry_msgs.msg.PointStamped) - robot frame
#
# Publishes:    
# '/stage/state/needle_pose'    (geometry_msgs.msg.PoseStamped)   - needle frame
# '/sensor/tip'                 (geometry_msgs.msg.PoseStamped)   - robot frame
# '/sensor/base'                (geometry_msgs.msg.PoseStamped)   - robot frame
# 'IGTL_STRING_OUT'             (ros2_igtl_bridge.msg.String)     - zFrame
# 'IGTL_POINT_OUT'              (ros2_igtl_bridge.msg.PointArray) - zFrame
# 
#########################################################################

class SmartNeedleInterface(Node):

    def __init__(self):
        super().__init__('smart_needle_interface')

        #Declare node parameters
        self.declare_parameter('use_slicer', False) # Push shape to OpenIGTLink bridge

#### Subscribed topics ###################################################

        #Topics from smart needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.shape_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        # Topics from keyboard interface node (robot initialization)
        self.subscription_initial_point = self.create_subscription(PoseStamped, '/stage/initial_point', self.initial_point_callback, 10)
        self.subscription_initial_point # prevent unused variable warning

#### Published topics ###################################################

        # Tip (robot frame)
        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)  #(stage frame)

        # Base (robot frame)
        timer_period_base = 0.3 # seconds
        self.timer_base = self.create_timer(timer_period_base, self.timer_base_callback)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10) #(stage frame)      

        # Base (needle frame)
        timer_period_needle_pose = 0.3 # seconds
        self.timer_needle_pose = self.create_timer(timer_period_needle_pose, self.timer_needle_pose_callback)        
        self.publisher_needle_pose = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)   #needle frame

        # Needle shape (zFrame)
        timer_period_shape = 1.0 # seconds
        self.timer_shape = self.create_timer(timer_period_shape, self.timer_shape_callback)        
        self.publisher_shapeheader = self.create_publisher(String, 'IGTL_STRING_OUT', 10)
        self.publisher_shape = self.create_publisher(PointArray, 'IGTL_POINT_OUT', 10)


#### Stored variables ###################################################

        self.needleToRobot = np.empty(shape=[0,7])  # Needle to robot frame transform
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform (from 3DSlicer)
        self.initial_point = np.empty(shape=[0,3])  # Robot position at begining of experiment
        self.stage = np.empty(shape=[0,3])          # Robot current position (robot frame)
        self.X = np.empty(shape=[0,7])              # Base pose (robot frame)
        self.needle_pose = np.empty(shape=[0,7])    # Base pose (needle frame)
        self.Z = np.empty(shape=[0,7])              # Tip pose (robot frame)
        self.sensorZ = np.empty(shape=[0,7])        # Tip pose (needle frame)

        # NeedleShape Bridge message
        self.shapecount = 0                         # Number of shape packace received
        self.shapeheader = None                     # Shape message header to push to 3D Slicer
        self.shapedata = None                       # Shape message data to push to 3D Slicer

        # Flag for pushing shape to bridge
        self.push_to_bridge = self.get_parameter('use_slicer').get_parameter_value().bool_value

#### Interface initialization ###################################################
        
        # Initialize zFrameToRobot transform
        # Fixed relation from geometry of robot and zFrame attachment
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Get initial robot pose
    def initial_point_callback(self, msg_robot):
        if (self.initial_point.size == 0): # Do it only once
            robot = msg_robot.pose
            # Set initial point
            self.initial_point = np.array([robot.position.x, robot.position.y, robot.position.z])
            q_tf1= np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
            q_tf2= np.quaternion(np.cos(np.deg2rad(45)), 0, 0, np.sin(np.deg2rad(45)))
            q_tf = q_tf1*q_tf2   
            # Set needleToRobot transform         
            self.needleToRobot = np.concatenate((self.initial_point[0:3], np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z]))) # Registration now comes from entry point

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        # Store current needle base pose (in robot and needle frames)
        if (self.needleToRobot.size != 0):
            needle_q = self.needleToRobot[3:7]
            self.X = np.array([self.stage[0], self.stage[1], self.stage[2], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) #base in robot frame       
            self.needle_pose = pose_inv_transform(self.X, self.needleToRobot)   # base in needle frame

    # Get current sensor measurements
    def shape_callback(self, msg_sensor):
        # From shape, get tip (last point)
        self.shapecount += 1
        shape = msg_sensor.poses      
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        self.sensorZ = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])
        # Transform from needle to robot frame
        if (self.needleToRobot.size != 0): 
            self.Z = pose_transform(self.sensorZ, self.needleToRobot)
            # Transform from needle to zFrame (to 3DSlicer)
            if self.push_to_bridge is True:
                # Build shape message to push to 3D Slicer
                frame_id = 'zFrame'
                timestamp = msg_sensor.header.stamp
                # Convert timestamp to a readable format
                now = datetime.datetime.now()
                timestamp_duration = datetime.timedelta(seconds=timestamp.nanosec / 1e9)
                duration_since_epoch = now - timestamp_duration
                # Get the time_t object from the datetime
                time_t_object = datetime.datetime.fromtimestamp(duration_since_epoch.timestamp())
                # Format the timestamp with seconds and milliseconds
                formatted_timestamp = time_t_object.strftime('%Y-%m-%d %H:%M:%S') + '.{:03d}'.format(int(timestamp.nanosec % 1e6 / 1e3))
                self.shapeheader = formatted_timestamp + ';' +str(self.shapecount) + ';'+ str(N) + ';' + frame_id
                # Get shape data points
                self.shapedata = []
                for pose in msg_sensor.poses:        
                    # Get next point and transform to zFrame
                    point_needle = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])  
                    point_robot = pose_transform(point_needle, self.needleToRobot)
                    point_zFrame = pose_inv_transform(point_robot, self.zFrameToRobot)
                    # Save it in point array
                    point = Point()
                    point.x = point_zFrame[0]
                    point.y = point_zFrame[1]
                    point.z = point_zFrame[2]
                    self.shapedata.append(point)

#### Publishing callbacks ###################################################

    # Publishes needle shape (robot frame) to IGTLink bridge
    def timer_shape_callback(self):
        if (self.shapedata is not None) and (self.shapeheader is not None):
            string_msg = String()
            string_msg.name = 'NeedleShapeHeader'
            string_msg.data = self.shapeheader
            pointarray_msg = PointArray()
            pointarray_msg.name = 'NeedleShapeZ'
            pointarray_msg.pointdata = self.shapedata
            # Push shape to IGTLBridge
            self.publisher_shapeheader.publish(string_msg)
            self.publisher_shape.publish(pointarray_msg)

    # Publishes needle base transformed to robot frame
    def timer_base_callback (self):
        # Publish last needle pose in robot frame
        if (self.X.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
            msg.pose.orientation = Quaternion(w=self.X[3], x=self.X[4], y=self.X[5], z=self.X[6])
            self.publisher_base.publish(msg)

    # Publishes needle tip transformed to robot frame
    def timer_tip_callback (self):
        # Publish last needle pose in robot frame
        if (self.Z.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
            msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
            self.publisher_tip.publish(msg)
            
    # Publishes needle displacement (x,y,z) in the needle coordinate frame
    def timer_needle_pose_callback (self):
        if (self.needle_pose.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            msg.pose.position = Point(x=self.needle_pose[0], y=self.needle_pose[1], z=self.needle_pose[2])
            msg.pose.orientation = Quaternion(w=self.needle_pose[3], x=self.needle_pose[4], y=self.needle_pose[5], z=self.needle_pose[6])
            self.publisher_needle_pose.publish(msg)
            
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

    smart_needle_interface = SmartNeedleInterface()
    rclpy.spin(smart_needle_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    smart_needle_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()