#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class TeleopJoy2:
    """Joystick teleoperation."""

    def __init__(self):
        """Create publisher and subscriber."""
        # publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                           Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        # subscriber
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        
        self.flying = False
    def joy_cb(self, msg):
        """Compute and publish twist command."""
        # initialize message
        
        # fill message
        
        if msg.buttons[7] > .5:
            if not self.flying:
                self.takeoff_pub.publish(Empty())
                self.flying = True
            
        else :
            self.cmd_vel_pub.publish(Twist())
            self.land_pub.publish(Empty())
            self.flying = False
       


def main():
    """Instantiate node and class."""
    # declare node
    rospy.init_node('teleop2')
    # instantiate class
    teleop_joy = TeleopJoy2()
    # run
    rospy.spin()


if __name__ == '__main__':
    main()
