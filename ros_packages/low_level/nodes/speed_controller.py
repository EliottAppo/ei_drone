#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class SpeedController:

    def __init__(self):

        #Publisher
        self.gathered_vel_pub = rospy.Publisher('/target_vel', Twist, queue_size=1)

        #Subscriber
        self.linear_x_sub = rospy.Subscriber('/linear_x', , self.linear_x_cb)
        self.linear_y_sub = rospy.Subscriber('/linear_y', Twist, self.linear_y_cb)
        self.linear_z_sub = rospy.Subscriber('/linear_z', Twist, self.linear_z_cb)
        self.angular_z = rospy.Subscriber('/angular_z', Twist, self.angular_z_cb)

    def linear_x_cb(self, msg):
    def linear_y_cb(self, msg):
    def linear_z_cb(self, msg):
        linear_z_gathered = msg.data
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def angular_z_cb(self, msg):
        angular_z_gathered = msg

def main():
    rospy.init_node('speed_controller')

    

if __name__ == '__main__':
    main()