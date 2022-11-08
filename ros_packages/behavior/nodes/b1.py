#! /usr/bin/env python3
import rospy
from behavior_class import Behavior

def main():
    rospy.init_node('behavior_b1')
    b1=Behavior('Hover')
    rospy.spin()

if __name__== '__main__':
    main()