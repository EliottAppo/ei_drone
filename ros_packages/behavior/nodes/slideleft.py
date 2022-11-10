#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32
from time import sleep



class SlideLeft(Behavior):
    
    def __init__(self):
        super().__init__("SlideLeft")
        self.sleep = None

        self.pub_linear_x = rospy.Publisher("/linear_x", Float32, queue_size=1)
        self.pub_angular_z = rospy.Publisher("/angular_z", Float32, queue_size=1)
        self.pub_linear_y = rospy.Publisher("/linear_y", Float32, queue_size=1)

    def on_status_on(self):       
        
        self.pub_linear_x.publish(0.5)
        self.pub_angular_z.publish(3)
        self.sleep = sleep(1)
        self.pub_linear_x.publish(0.0)
        self.pub_angular_z.publish(0.0)
        self.pub_linear_y.publish(-0.5)


def main():
    rospy.init_node("SlideLeft")
    SlideLeft()
    rospy.spin()


if __name__ == "__main__":
    main()
