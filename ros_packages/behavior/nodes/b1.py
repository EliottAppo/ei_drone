#! /usr/bin/env python3
import rospy
from behavior_class import Behavior

#def main():
class B1(Behavior):

    def __init__(self):
        super().__init__('b1')

    #rospy.init_node('behavior_b1')
    # b1=Behavior('Hover')
    # rospy.spin()


def main():
	rospy.init_node('b1')
	b1 = B1()
	rospy.spin()

if __name__ == '__main__':
    main()