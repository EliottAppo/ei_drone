#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multiprocessing import Lock
from slideleft import SlideLeft
from visual_processing.msg import VanishingPoint


class TurnLeft(SlideLeft):
    def __init__(self):
        super().__init__("TurnLeft")
        self.detect_corridor = False
        self.detect_corridor_mutex = Lock()
        # 'self.vp_count' stores how many vanishing points were detected consecutively, while
        # 'self.vp_min_count' is the minimum necessary value to consider that a corridor was
        # detected
        self.vp_count = 0
        self.vp_count_mutex = Lock()
        self.vp_min_count = 20

        self.sub_vanishing_point = rospy.Subscriber(
            '/vanishing_point/point_result', VanishingPoint, self.vanishing_point_cb)
        self.pub_command = rospy.Publisher('/command', String, queue_size=1)

    def vanishing_point_cb(self, msg):
        with self.vp_count_mutex:
            if msg.vp_exists:
                self.vp_count += 1
            else:  # If we receive at least one vp_exists = False, makes the counter go back to 0
                self.vp_count = 0
        if self.vp_count >= self.vp_min_count:
            with self.corridor_detected_mutex:
                self.corridor_detected = True

    def get_corridor_detected(self):
        with self.detect_corridor_mutex:
            return self.detect_corridor

    def reset_detection(self):
        with self.vp_count_mutex:
            self.vp_count = 0
        with self.corridor_detected_mutex:
            self.corridor_detected = False

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.get_corridor_detected():
                    self.reset_detection()
                    self.inactivate()
                    self.pub_command.publish('MoveVP')
                else:
                    # NOTE: this is in open loop. In closed loop we have to get x,y,z speeds from the loop feedback
                    self.linear_x_pub.publish(0)
                    self.linear_y_pub.publish(-0.5)
                    self.angular_z_pub.publish(0)
                    rospy.sleep(0.1)


def main():
    rospy.init_node("TurnLeft")
    behavior = TurnLeft()
    behavior.loop()


if __name__ == "__main__":
    main()
