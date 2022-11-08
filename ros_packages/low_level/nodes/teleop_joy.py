#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class TeleopJoy:
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
        cmd_vel_msg = Twist()
        # fill message
        
        if msg.buttons[7] > .5:
            if not self.flying:
                self.takeoff_pub.publish(Empty())
                self.flying = True
            else:

                cmd_vel_msg.linear.x = msg.axes[1]
                cmd_vel_msg.linear.y = msg.axes[2]
                cmd_vel_msg.linear.z = msg.axes[3]
                cmd_vel_msg.angular.x = 0
                cmd_vel_msg.angular.y = 0
                cmd_vel_msg.angular.z =  msg.axes[0]
        else :
            self.land_pub.publish(Empty())
            self.flying = False
        # publish message
        self.cmd_vel_pub.publish(cmd_vel_msg)


def main():
    """Instantiate node and class."""
    # declare node
    rospy.init_node('teleop_joy')
    # instantiate class
    teleop_joy = TeleopJoy()
    # run
    rospy.spin()


if __name__ == '__main__':
    main()
