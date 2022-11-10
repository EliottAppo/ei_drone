#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32
from time import sleep
from nav_msgs.msg import Odometry
from math import atan2



class SlideLeft(Behavior):
    
    def __init__(self):
        super().__init__("SlideLeft")
        self.sleep = None

        self.pub_linear_x = rospy.Publisher("/linear_x", Float32, queue_size=1)
        self.pub_angular_z = rospy.Publisher("/angular_z", Float32, queue_size=1)
        self.pub_linear_y = rospy.Publisher("/linear_y", Float32, queue_size=1)
        self.sub_odom = rospy.Subscriber("/bebop/odom", Odometry, self.cb_odom)

        self.odom_mutex = Lock()

    def cb_odom(self, msg):
        with self.odom_mutex:
            self.current_quaternion = msg.pose.pose.orientation

    
    def on_status_on(self):       
        
        qx, qy, qz, qw = self.current_quaternion
        self.line = [qw**2 + qx**2 - qy**2 - qz**2, 2*qw*qz + 2*qx*qy]

        # il faut faire un loop qui pour chaque itération calcule theta et dist, et envoie les commandes sur /linear_x et /angular_z en conséquence
        # utiliser rviz pour voir en temps réel l'axes et les commandes pour le débug

        self.pub_linear_x.publish(0.5)
        self.pub_angular_z.publish(3)
        self.sleep = sleep(1)
        self.pub_linear_x.publish(0.0)
        self.pub_angular_z.publish(0.0)


        #self.pub_linear_y.publish(-0.5)


def main():
    rospy.init_node("SlideLeft")
    SlideLeft()
    rospy.spin()


if __name__ == "__main__":
    main()
