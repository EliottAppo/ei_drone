#! /usr/bin/env python3
import rospy
from behavior_class import Behavior

#def main():
#    rospy.init_node('behavior_b2')
#    b2=Behavior('Forward')
#    rospy.spin()
class B2(Behavior):

    def __init__(self):
        super().__init__('b2')

def main():
	rospy.init_node('b2')
	b2 = B2()
	rospy.spin()

if __name__ == '__main__':
    main()