import getch
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int8

#########################################################################
#
# Keypress
#
# Description:
# This node runs the keyboard user interface with the SmartTemplate
#
# Publishes:   
# '/keyboard/key'               (std_msgs.msg.Int8)
#
##########################################################################

class Keypress(Node):

    def __init__(self):
        super().__init__('keypress')

#### Published topics ###################################################

        self.publisher = self.create_publisher(Int8, '/keyboard/key', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_keyboard_callback)

#### Publishing callbacks ###################################################

    def timer_keyboard_callback(self):
        k = ord(getch.getch())  # this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        # to filter only desired keys: 10=ENTER, 32=SPACE, 50=down_numkey, 52=left_numkey, 54=right_numkey, 56=up_numkey
        # 65=A, 72=H, 82=R
        if (k==10) or (k==32) or (k==50) or (k==52) or (k==54) or (k==56) or (k==65) or (k==72) or (k==82):
            msg = Int8()
            msg.data = k
            key=''
            if (k==10):
                key='ENTER'
            elif (k==32):
                key='SPACE'
            elif (k==50):
                key='DOWN'
            elif (k==52):
                key='LEFT'
            elif (k==54):
                key='RIGHT'
            elif (k==56):
                key='UP'
            elif (k==65):
                key='ABORT'
            elif (k==72):
                key='HOME'
            elif (k==82):
                key='RETRACT'
            self.publisher.publish(msg)
            self.get_logger().info('Pressed %s' %(key))

########################################################################

def main(args=None):
    rclpy.init(args=args)

    keypress = Keypress()
    keypress.get_logger().info('*** Keyboard User Interface ***')

    rclpy.spin(keypress)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keypress.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()