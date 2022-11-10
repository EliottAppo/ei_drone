#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from multiprocessing import Lock
from slideleft import SlideRight


class DoorRight(SlideRight):
    def __init__(self):
        self.door_exists = False
        self.door_exists_mutex = Lock()
        self.sub_door_exists = rospy.Subscriber(
            '/door_exists', Bool, self.door_exists_cb)
        self.pub_command = rospy.Publisher('/command', String, queue_size=1)
        self.door_count = 0
        self.door_count_mutex = Lock()
        self.door_min_count = 20

    def door_exists_cb(self, msg):
        with self.door_count_mutex:
            if msg.data:
                self.door_count += 1
        if self.door_count >= self.door_min_count:
            with self.corridor_detected_mutex:
                self.corridor_detected = True

    def door_existed(self):
        with self.door_exists_mutex:
            return self.door_exists

    def reset_detection(self):
        with self.door_count_mutex:
            self.door_count = 0
        with self.door_exists_mutex:
            self.door_exists = False

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.door_existed():
                    self.reset_detection()
                    self.inactivate()
                    self.pub_command.publish('CrossDoor')
            else:
                # NOTE: this is in open loop. In closed loop we have to get x,y,z speeds from the loop feedback
                self.linear_x_pub.publish(0)
                self.linear_y_pub.publish(-0.5)
                self.angular_z_pub.publish(0)
                rospy.sleep(0.1)


def main():
    rospy.init_node("DoorRight")
    behavior = DoorRight()
    behavior.loop()


if __name__ == "__main__":
    main()
