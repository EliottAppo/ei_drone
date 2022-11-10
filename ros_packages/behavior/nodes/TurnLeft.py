#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multiprocessing import Lock
from slideleft import SlideLeft



class TurnLeft(SlideLeft):
    def __init__(self):
        super().__init__("TurnLeft")
        self.detect_corridor = False
        self.detect_corridor = Lock()
        # 'self.vp_count' stores how many vanishing points were detected consecutively, while
        # 'self.vp_min_count' is the minimum necessary value to consider that a corridor was
        # detected
        self.vp_count = 0
        self.vp_count_mutex = Lock()
        self.vp_min_count = rospy.get_param('logical_behaviors/Turn/vp_min_count')
        
        self.sub_vanishing_point = rospy.Subscriber('/vanishing_point', VanishingPoint, self.vanishing_point_cb)
        self.pub_command = rospy.Publisher('/command', String, queue_size=1)

    def on_status_on(self):
        self.reset_detection()
        return super().on_status_on()


    def vanishing_point_cb(self, msg):
        with self.vp_count_mutex:
            if self.get_finished_open_loop():
                if msg.vp_exists:
                    self.vp_count += 1
                else:  # If we receive at least one vp_exists = False, makes the counter go back to 0
                    self.vp_count = 0
        if self.vp_count >= self.vp_min_count:
            with self.corridor_detected_mutex:
                self.corridor_detected = True

    def get_corridor_detected(self):
        with self.corridor_detected_mutex:
            return self.corridor_detected
    
    def reset_detection(self):
        with self.vp_count_mutex:
            self.vp_count = 0
        with self.corridor_detected_mutex:
            self.corridor_detected = False

    def loop(self, freq=10):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.get_corridor_detected():
                    self.reset_detection()
                    self.deactivate()
                    self.command_pub.publish('GoAhead')
                else:
                    command_linear_x = self.get_command_linear_x()
                    command_linear_y = self.get_command_linear_y()
                    command_angular_z = self.get_command_angular_z()
                    self.linear_x_pub.publish(command_linear_x)
                    self.linear_y_pub.publish(command_linear_y)
                    self.angular_z_pub.publish(command_angular_z)
                    rospy.sleep(1/freq) # we sleep for 1/freq

