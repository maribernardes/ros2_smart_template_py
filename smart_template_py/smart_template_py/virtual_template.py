import rclpy
import numpy as np
import time
import math

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


TIMEOUT = 5             # timeout (sec) for move_and_observe action server 

#########################################################################
#
# Virtual Template
#
# Description:
# This node implements a virtual node to emulate the SmartTemplate
# Implements the robot service and action servers
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

class VirtualSmartTemplate(Node):

    def __init__(self):
        super().__init__('virtual_smart_template')      

    #### Get joint limits from robot description ###################################################
        self.declare_parameter('robot_description', '')
        urdf_str = self.get_parameter('robot_description').get_parameter_value().string_value
        self.joint_names = []
        self.joint_limits = {}
        self.joint_channels = {}
        self.joint_mm_to_count = {}
        self.joint_count_to_mm = {}
        try:
            root = ET.fromstring(urdf_str)
            for joint_elem in root.findall('joint'):
                name = joint_elem.attrib.get('name')
                if not name:
                    continue
                # === Limits ===
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None and 'lower' in limit_elem.attrib and 'upper' in limit_elem.attrib:
                    try:
                        lower = float(limit_elem.attrib['lower']) * 1000  # Convert to mm
                        upper = float(limit_elem.attrib['upper']) * 1000
                        self.joint_names.append(name)
                        self.joint_limits[name] = {
                            'lower': lower,
                            'upper': upper
                        }
                        self.get_logger().info(f"{name} limits [mm]: lower={lower}, upper={upper}")
                    except ValueError:
                        self.get_logger().warn(f"Invalid limit values for joint '{name}'")
                # === Channel ===
                channel_elem = joint_elem.find('channel')
                if channel_elem is not None:
                    channel = channel_elem.text.strip()
                    self.joint_channels[name] = channel
                    self.get_logger().info(f"{name} is assigned to channel '{channel}'")
                # === mm_to_count and count_to_mm ===
                mm_to_count_elem = joint_elem.find('mm_to_count')
                if mm_to_count_elem is not None:
                    try:
                        mm_to_count = float(mm_to_count_elem.text.strip())
                        self.joint_mm_to_count[name] = mm_to_count
                        if mm_to_count != 0.0:
                            self.joint_count_to_mm[name] = 1.0 / mm_to_count
                            self.get_logger().info(f"{name} mm_to_count = {mm_to_count}, count_to_mm = {1.0/mm_to_count}")
                        else:
                            self.get_logger().warn(f"{name} has mm_to_count = 0.0 — cannot compute inverse.")
                    except ValueError:
                        self.get_logger().warn(f"Invalid mm_to_count value for joint '{name}'")
        except ET.ParseError as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")

        # Sort joints by channel order (A, B, C, ...)
        sorted_joint_items = sorted(self.joint_channels.items(),key=lambda item: item[1])   # (joint_name, channel) sort by channel letter        
        self.joint_names = [joint for joint, _ in sorted_joint_items]                       # Ordered list of joint_names
        self.joint_channels = OrderedDict((joint, self.joint_channels[joint]) for joint in self.joint_names)
        self.joint_limits = OrderedDict((joint, self.joint_limits[joint]) for joint in self.joint_names if joint in self.joint_limits)
        self.joint_mm_to_count = OrderedDict((joint, self.joint_mm_to_count[joint]) for joint in self.joint_names if joint in self.joint_mm_to_count)
        self.joint_count_to_mm = OrderedDict((joint, self.joint_count_to_mm[joint]) for joint in self.joint_names if joint in self.joint_count_to_mm)

        for joint in self.joint_names:
            limits = self.joint_limits.get(joint)
            if limits:
                self.get_logger().info(
                    f"  {joint}: lower = {limits['lower']:.2f} mm, upper = {limits['upper']:.2f} mm"
                )
            else:
                self.get_logger().info(f"  {joint}: No limits defined.")

#### Published topics ###################################################

        # Current position
        timer_period_stage = 0.3  # seconds
        self.timer_stage = self.create_timer(timer_period_stage, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PointStamped, '/stage/state/guide_pose', 10)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

#### Subscribed topics ###################################################
        self.subscription_desired_position = self.create_subscription(Point, '/desired_position', self.desired_position_callback, 10)
        self.subscription_desired_position # prevent unused variable warning

#### Action/Service server ##############################################

        self._action_server = ActionServer(self, MoveAndObserve, '/stage/move_and_observe', execute_callback=self.execute_move_and_observe_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.move_and_observe_callback, cancel_callback=self.cancel_move_and_observe_callback)
        self.command_server = self.create_service(Command, '/stage/command', self.command_callback, callback_group=ReentrantCallbackGroup())
        self.move_server = self.create_service(Move, '/stage/move', self.move_callback, callback_group=ReentrantCallbackGroup())
        self.current_position_server = self.create_service(GetPoint, '/stage/get_position', self.current_position_callback, callback_group=ReentrantCallbackGroup())

#### Node initialization ###################################################

        # Initial home position - currently initializing in (0,0,0)
        #X = Horizontal
        #Y = Insertion
        #Z = Vertical
        self.position = np.array([0.0, 0.0, 0.0])
        self.desired_position = np.array([0.0, 0.0, 0.0])

        # Motion step for simulation
        self.motion_step = np.array([0.05, 0.1, 0.05])

        # Flag to abort command
        self.abort = False   

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})


#### Internal functions ###################################################

    # Get current robot position error
    def get_error(self):
        err = self.position - self.desired_position
        self.get_logger().info('Error: X = (%.2f mm), Y = (%.2f mm), Z = (%.2f mm)' %(err[0], err[1], err[2]))
        return [err[0], err[1], err[2]]
    
    # Get current robot position error
    def get_position(self):
        return np.copy(self.position)
    
    # Abort any ongoing motion (stop where it is)
    def abort_motion(self):
        self.desired_position = self.position

    # Check is a joint value [mm] is within the specified limit. If not, caps it to limit
    def check_limits(self, joint_value: float, joint_name: str) -> float:
        limits = self.joint_limits.get(joint_name)
        if not limits:
            self.get_logger().warn(f"No limits found for joint '{joint_name}'. Using raw value.")
            return joint_value
        lower = limits['lower']
        upper = limits['upper']
        if joint_value < lower or joint_value > upper:
            self.get_logger().warn(
                f"{joint_name} value {joint_value:.2f} mm out of bounds. "
                f"Capping to [{lower:.2f}, {upper:.2f}]"
            )
        return max(lower, min(joint_value, upper))

    # Sends a movement command to all 3 joints based on the goal [x, y, z] in mm
    def send_movement(self, goal):
        if len(goal) != 3:
            self.get_logger().error("Goal must be a list of [x, y, z] in mm.")
            return
        self.desired_position = goal

    # Calculate euclidean error from each axis error
    def error_3d(self, err):
        return math.sqrt(err[0]**2+(err[1])**2+err[2]**2)

    # Emulate robot motion
    def emulate_motion(self):
        delta = self.desired_position-self.position
        self.get_logger().info('delta = %s' %delta)
        step = np.minimum(np.absolute(delta), self.motion_step)
        self.position = self.position + np.multiply(np.sign(delta), step)

#### Listening callbacks ###################################################

    # A request for desired position was sent
    def desired_position_callback(self, msg):
        goal = np.array([msg.x, msg.y, msg.z])
        #goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.get_logger().info(f'Received request: x={goal[0]}, y={goal[1]}, z={goal[2]}')
        self.send_movement(goal)
    '''    
    # A request for desired position was sent
    def desired_position_callback(self, msg):
        #goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Desired position: x={goal[0]}, y={goal[1]}, z={goal[2]}')
        # Create request
        request = Move.Request()
        request.x = goal[0]
        request.y = goal[1]
        request.z = goal[2]
        request.eps = 0.5
        # Directly call the service callback (no need for a client)
        response = Move.Response()
        response = self.move_callback(request, response)
        self.get_logger().info(f"Service returned: {response.response}")


        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        # When stage request done, do callback
        future.add_done_callback(partial(self.update_stage_callback))
        return future.result()
    '''
#### Publishing callbacks ###################################################

    # Timer to publish '/stage/state/pose'  
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
        # Update joint_state message to publish
        joint_state_msg = JointState()                
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [0.001*position[0], 0.001*position[1], 0.001*position[2]] # Convert from mm to m
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
            goal = np.array([request.x, request.y, request.z])
            self.send_movement(goal)
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
    # This action server allows multiple goals in parallel
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
        self.send_movement(goal) 
        ####### Feedback loop (while goal is not reached or not timeout)
        # TODO or not TODO: Update feedback message
        start_time = time.time()
        result.error_code = 0
        while True:
            time.sleep(0.1)
            self.emulate_motion()
            # Check current position error
            err = self.error_3d(self.get_error())
            if self.abort == True:
                # self.galil.GCommand('SH')
                goal_handle.abort()
                result.error_code = 2   # abort
                self.get_logger().info('Chegou no abort')
                self.abort = False
                break
           # Check if reached target
            if err <= my_goal.eps:
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
        result.error = err
        result.time = time.time()-start_time
        self.get_logger().info('Finished move_and_observe')
        return result
       
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
