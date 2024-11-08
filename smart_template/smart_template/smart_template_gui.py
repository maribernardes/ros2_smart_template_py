import threading
import uuid  # Import uuid to generate unique IDs

# Import ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import rqt Plugin base class
from rqt_gui_py.plugin import Plugin

# Import Qt libraries
from python_qt_binding.QtWidgets import (
    QWidget, QLabel, QSlider, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QSpacerItem, QFrame, QGridLayout)
from python_qt_binding.QtCore import Qt, Signal, Slot, QTimer

# Import ROS 2 message and service types
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from smart_template_interfaces.action import MoveStage
from smart_template_interfaces.srv import ControllerCommand

from functools import partial

class SmartTemplateGUIPlugin(Plugin):
    # Define a signal that carries joint names and positions
    update_joint_state_signal = Signal(dict)

    def __init__(self, context):
        super(SmartTemplateGUIPlugin, self).__init__(context)
        self.setObjectName('SmartTemplateGUIPlugin')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setWindowTitle('SmartTemplate GUI')
        context.add_widget(self._widget)

        # Initialize ROS 2 node if not already initialized
        if not rclpy.ok():
            rclpy.init(args=None)
        unique_id = uuid.uuid4().hex[:8]
        self.node = rclpy.create_node(f'smart_template_gui_{unique_id}')

        # Define Joint Names and Limits Before Subscribers
        self.joint_names = ['horizontal_joint', 'vertical_joint', 'insertion_joint']
        self.joint_limits = {
            'horizontal_joint': {'min': -25.0, 'max': 25.0},  # mm
            'vertical_joint': {'min': -30.0, 'max': 30.0},    # mm
            'insertion_joint': {'min': 0.0, 'max': 200.0},         # mm
        }

        # Initialize joint state variables
        self.current_joint_states = {name: 0.0 for name in self.joint_names}
        self.desired_joint_values = {name: 0.0 for name in self.joint_names}

        # Robot status
        self.robot_idle = True

        # Start a QTimer to spin the node
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)  # Call spin_once every 10 milliseconds

        # ROS subscribers
        self.joint_state_subscriber = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # ROS Action client
        self.action_client = ActionClient(self.node, MoveStage, '/stage/move')
        self.node.get_logger().info('Waiting for action server /stage/move...')
        self.action_client.wait_for_server()

        # ROS Service client
        self.service_client = self.node.create_client(ControllerCommand, '/stage/command')
        self.node.get_logger().info('Waiting for service /stage/command...')
        self.service_client.wait_for_service()

        # Setup UI elements
        self.setup_ui()

        # Connect the signal to the slot
        self.update_joint_state_signal.connect(self.update_joint_state_gui)

        self.node.get_logger().info('Successfully connected to SmartTemplate')

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def setup_ui(self):
        # Main layout to hold left and right panels
        main_layout = QHBoxLayout()

        # Left panel: Joint controls
        joint_controls_layout = QVBoxLayout()

        # Dictionaries to hold widgets
        self.sliders = {}
        self.text_boxes = {}
        self.current_value_boxes = {}  # For 'Current [mm]' textboxes

        for joint in self.joint_names:
            # Remove '_joint' suffix and capitalize the label
            joint_label = QLabel(f'{joint.replace("_joint", "").capitalize()}')

            # Slider for current joint state
            slider = QSlider(Qt.Horizontal)
            limits = self.joint_limits[joint]
            slider.setMinimum(int(limits['min'] * 100))
            slider.setMaximum(int(limits['max'] * 100))
            slider.setEnabled(False)  # Non-editable
            slider.setValue(0)

            # Min and Max labels
            min_label = QLabel(f"{limits['min']}")
            max_label = QLabel(f"{limits['max']}")

            # Layout for slider and labels
            slider_layout = QHBoxLayout()
            slider_layout.addWidget(min_label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(max_label)

            # Text box for current joint value (non-editable)
            current_value_box = QLineEdit('0.0')
            current_value_box.setReadOnly(True)
            current_value_box.setStyleSheet("background: transparent; border: none; color: black;")
            current_value_label = QLabel('Current [mm]:')

            # Text box for desired joint value
            desired_value_box = QLineEdit('0.0')
            desired_value_label = QLabel('Desired [mm]:')

            # Layout for current and desired values
            value_layout = QHBoxLayout()
            value_layout.addWidget(current_value_label)
            value_layout.addWidget(current_value_box)
            value_layout.addWidget(desired_value_label)
            value_layout.addWidget(desired_value_box)

            # Layout for each joint
            joint_layout = QVBoxLayout()
            joint_layout.addWidget(joint_label)
            joint_layout.addLayout(slider_layout)
            joint_layout.addLayout(value_layout)

            # Add the joint layout to the main joint controls layout
            joint_controls_layout.addLayout(joint_layout)

            # Add a horizontal separator line after each joint section
            separator_line = QFrame()
            separator_line.setFrameShape(QFrame.HLine)
            separator_line.setFrameShadow(QFrame.Sunken)
            joint_controls_layout.addWidget(separator_line)

            # Save references
            self.sliders[joint] = slider
            self.text_boxes[joint] = desired_value_box
            self.current_value_boxes[joint] = current_value_box

        # Add a "Send" button at the bottom of the left panel
        send_button = QPushButton('Send')
        send_button.clicked.connect(self.send_desired_joint_values)  # Connect button to the function
        joint_controls_layout.addWidget(send_button)

        # Add joint controls to main layout
        main_layout.addLayout(joint_controls_layout)

        # Add spacer between left and right panels
        main_layout.addSpacerItem(QSpacerItem(15, 0))  # Horizontal spacer to increase space

        # Vertical line separator
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line)
        main_layout.addSpacerItem(QSpacerItem(15, 0))  # Horizontal spacer to increase space

        # Right panel: Button controls
        button_controls_layout = QVBoxLayout()

        # Top row: RETRACT and HOME buttons
        retract_home_layout = QHBoxLayout()
        retract_button = QPushButton('RETRACT')
        home_button = QPushButton('HOME')

        # Connect buttons to the service request function
        retract_button.clicked.connect(lambda: self.send_service_request('RETRACT'))
        home_button.clicked.connect(lambda: self.send_service_request('HOME'))

        retract_home_layout.addWidget(retract_button)
        retract_home_layout.addWidget(home_button)
        button_controls_layout.addLayout(retract_home_layout)

        # Middle row: Cross-patterned arrow buttons
        cross_pattern_layout = QVBoxLayout()

        # Create directional buttons
        up_button = QPushButton('↑')
        left_button = QPushButton('←')
        right_button = QPushButton('→')
        down_button = QPushButton('↓')

        # Create step size text boxes
        self.up_down_step_size = QLineEdit('1.0')  # Text box for up/down step size
        self.left_right_step_size = QLineEdit('1.0')  # Text box for left/right step size
        self.up_down_step_size.setFixedWidth(50)
        self.left_right_step_size.setFixedWidth(50)

        # Set fixed size for buttons
        button_size = 40
        up_button.setFixedSize(button_size, button_size)
        left_button.setFixedSize(button_size, button_size)
        right_button.setFixedSize(button_size, button_size)
        down_button.setFixedSize(button_size, button_size)

        # Connect buttons to the unified motion handling function
        up_button.clicked.connect(lambda: self.handle_step_motion_button('UP'))
        left_button.clicked.connect(lambda: self.handle_step_motion_button('LEFT'))
        right_button.clicked.connect(lambda: self.handle_step_motion_button('RIGHT'))
        down_button.clicked.connect(lambda: self.handle_step_motion_button('DOWN'))

        # Layout for cross pattern with empty center
        cross_layout = QGridLayout()
        cross_layout.setAlignment(Qt.AlignCenter)  # Center the grid

        # Position elements in the grid
        cross_layout.addWidget(up_button, 0, 1, alignment=Qt.AlignCenter)  # Up button
        cross_layout.addWidget(left_button, 1, 0, alignment=Qt.AlignCenter)  # Left button
        cross_layout.addWidget(right_button, 1, 2, alignment=Qt.AlignCenter)  # Right button
        cross_layout.addWidget(down_button, 2, 1, alignment=Qt.AlignCenter)  # Down button

        # Add step size text boxes near corresponding buttons
        cross_layout.addWidget(self.up_down_step_size, 0, 3, alignment=Qt.AlignLeft)  # Step size for up/down
        cross_layout.addWidget(self.left_right_step_size, 1, 3, alignment=Qt.AlignLeft)  # Step size for left/right

        # Add cross layout to main button controls layout
        cross_pattern_layout.addLayout(cross_layout)
        button_controls_layout.addLayout(cross_pattern_layout)

        # Bottom row: + and - buttons
        plus_minus_layout = QHBoxLayout()
        plus_button = QPushButton('+')
        minus_button = QPushButton('-')
        self.insertion_step_size = QLineEdit('5.0')  # Text box for insertion step size
        self.insertion_step_size.setFixedWidth(50)

        # Connect buttons to the unified motion handling function
        plus_button.clicked.connect(lambda: self.handle_step_motion_button('+'))
        minus_button.clicked.connect(lambda: self.handle_step_motion_button('-'))

        plus_minus_layout.addWidget(plus_button)
        plus_minus_layout.addWidget(minus_button)
        plus_minus_layout.addStretch()  # Spacer to push the text box to the right
        plus_minus_layout.addWidget(self.insertion_step_size)

        button_controls_layout.addLayout(plus_minus_layout)

        # Add button controls to main layout
        main_layout.addLayout(button_controls_layout)

        # Set layout to the widget
        self._widget.setLayout(main_layout)

    def handle_step_motion_button(self, direction):
        try:
            step_size = 0
            joint_key = ''

            if direction in ['UP', 'DOWN']:
                step_size = float(self.up_down_step_size.text())
                joint_key = 'vertical_joint'
                step_modifier = 1 if direction == 'UP' else -1
            elif direction in ['LEFT', 'RIGHT']:
                step_size = float(self.left_right_step_size.text())
                joint_key = 'horizontal_joint'
                step_modifier = 1 if direction == 'RIGHT' else -1
            elif direction in ['+', '-']:
                step_size = float(self.insertion_step_size.text())
                joint_key = 'insertion_joint'
                step_modifier = 1 if direction == '+' else -1

            if joint_key:
                self.desired_joint_values[joint_key] += step_modifier * step_size
                self.send_action_request(self.desired_joint_values)
        except ValueError:
            self.node.get_logger().warn('Invalid step size value')


    def joint_state_callback(self, msg):
        try:
            joint_states = {}
            for name, position in zip(msg.name, msg.position):
                if name in self.joint_names:
                    # Convert position from meters to millimeters
                    position_mm = position * 1000.0
                    joint_states[name] = position_mm
            # Emit the signal with the joint states
            self.update_joint_state_signal.emit(joint_states)
        except Exception as e:
            self.node.get_logger().error(f'Error in joint_state_callback: {e}')

    @Slot(dict)
    def update_joint_state_gui(self, joint_states):
        for name, position in joint_states.items():
            if name in self.joint_names:
                self.current_joint_states[name] = position
                # Update slider value
                slider = self.sliders[name]
                slider_value = int(position * 100)
                # Ensure slider_value is within slider range
                slider_min = slider.minimum()
                slider_max = slider.maximum()
                slider_value = max(min(slider_value, slider_max), slider_min)
                slider.setValue(slider_value)
                # Update the 'Current [mm]' textbox
                current_value_box = self.current_value_boxes[name]
                current_value_box.setText(f'{position:.2f}')
        # You can also update any other GUI elements if needed

    def send_desired_joint_values(self):
        # Get desired values from text boxes
        valid_input = True
        for joint in self.joint_names:
            textbox = self.text_boxes[joint]
            try:
                value = float(textbox.text())
                limits = self.joint_limits[joint]
                if limits['min'] <= value <= limits['max']:
                    self.desired_joint_values[joint] = value
                else:
                    self.node.get_logger().warn(
                        f'Desired value for joint {joint} out of limits')
                    valid_input = False
            except ValueError:
                self.node.get_logger().warn(
                    f'Invalid input for joint {joint}')
                valid_input = False

        if valid_input:
            # Send action request
            self.send_action_request(self.desired_joint_values)


    # Send service request to smart_template robot 
    def send_service_request(self, cmd_string):
        self.robot_idle = False
        request = ControllerCommand.Request()
        request.command = cmd_string
        future = self.service_client.call_async(request)
        future.add_done_callback(partial(self.get_response_callback))

    def get_response_callback(self, future):
        try:
            response = future.result()
            self.node.get_logger().info('Service call sucessful: %s' %(response.response)) 
        except Exception as e:
            self.node.get_logger().error('Service call failed: %r' %(e,))
        self.robot_idle = True

    # Send action request to smart_template robot
    def send_action_request(self, desired_joint_values):
        self.robot_idle = False
        # Send command to stage
        goal_msg = MoveStage.Goal()
        # Use values in millimeters as the action server expects
        goal_msg.x = desired_joint_values['horizontal_joint']
        goal_msg.y = desired_joint_values['insertion_joint']
        goal_msg.z = desired_joint_values['vertical_joint']
        goal_msg.eps = 0.5  # in mm (as per your robot's expectation)
        self.node.get_logger().info(
            f'Sending goal: x={goal_msg.x} mm, y={goal_msg.y} mm, z={goal_msg.z} mm')

        # Send goal asynchronously
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().info('Goal rejected :(')
                return
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.node.get_logger().error(f'Error in goal_response_callback: {e}')

    def get_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info('Goal succeeded')
            else:
                self.node.get_logger().info('Goal failed with status: {}'.format(status))
            self.robot_idle = True  # Set robot status to IDLE
        except Exception as e:
            self.node.get_logger().error(f'Error in get_result_callback: {e}')

    def feedback_callback(self, feedback_msg):
        # Handle feedback if needed
        feedback = feedback_msg.feedback
        # For example, update GUI elements or log feedback
        pass

    def shutdown_plugin(self):
        # Shutdown QTimer
        self.timer.stop()
        # Shutdown ROS node
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
