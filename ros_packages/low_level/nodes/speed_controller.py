#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from multiprocessing import Lock
import numpy as np

class SpeedController:

    def __init__(self):
        #self.x_vel_max = get_param()


        #Publisher
        self.gathered_vel_pub = rospy.Publisher('/target_vel', Twist, queue_size=1)
        
        #Subscriber
        self.linear_x_sub = rospy.Subscriber('/linear_x', Float32, self.linear_x_cb)
        self.linear_y_sub = rospy.Subscriber('/linear_y', Float32, self.linear_y_cb)
        self.linear_z_sub = rospy.Subscriber('/linear_z', Float32, self.linear_z_cb)
        self.angular_z = rospy.Subscriber('/angular_z', Float32, self.angular_z_cb)

        self.hover_mode = rospy.Subscriber('/hover', Empty, self.hover_cb)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        #Variables
        self.gathered_vel_msg = Twist()
        self.gathered_vel_msg_mutex = Lock()
        self.hover = True

    def odom_cb(self, msg):
        current_velocity = msg.twist.twist

    def hover_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg = Twist()
            self.hover = True
        
    def linear_x_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.x = msg.data  
            self.hover = False

    def linear_y_cb(self, msg):
        
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.y = msg.data 
            self.hover = False 
    def linear_z_cb(self, msg):
      
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.z = msg.data 
            self.hover = False      

    def angular_z_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.angular.z = msg.data
            self.hover = False

    def loop(self):
        while not rospy.is_shutdown():
            with self.gathered_vel_msg_mutex:
                if self.hover:
                    self.gathered_vel_pub.publish(Twist())
                else:
                        
                    self.gathered_vel_output = Twist()
                    self.gathered_vel_output.linear.z = self.gathered_vel_msg.linear.z
                    self.gathered_vel_output.angular.z = self.gathered_vel_msg.angular.z
                    self.gathered_vel_pub.publish(self.gathered_vel_output)

def main():
    """Instantiate node and class."""
    # declare node
    rospy.init_node('speed_controller')
    # instantiate class
    speed_controller = SpeedController()
    speed_controller.loop()
   

    

if __name__ == '__main__':
    main()