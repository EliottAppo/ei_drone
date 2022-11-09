#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from multiprocessing import Lock
from behavior.msg import BehaviorStatus
from time import time
from std_msgs.msg import String

class Scheduler:
    def __init__(self,pub_behavior):
        self.queue = []
        self.time = None
        self.pub_behavior=pub_behavior


    def create_schedule(self,events):
        self.time = time()
        for behavior in events:
            self.queue.append(behavior)

    def publish(self):
        for behavior in self.queue:
            current_time = time()
            date, name = behavior
            if current_time-self.time > date:
                msg = BehaviorStatus()
                msg.name = name
                msg.status = True
                self.pub_behavior.publish(msg)
                self.queue.remove(self.queue[0])


        
 

class Command:
    def __init__(self):
        self.behaviors = ['TakeOff','Hover', 'MoveLeft',
                          'MoveUp', 'MoveRight', 'RotateLeft']
        self.commands = {
            'TakeOff': [(0,'TakeOff')],
            'Stop': [(0, 'Hover')],
            'Dance': [(0, 'MoveLeft'), (0, 'MoveUp'), (2.5, 'MoveRight'), (3.2, 'Hover')]
        }
        self.sub_command = rospy.Subscriber(
            '/command', String, self.command_cb, queue_size=1, buff_size=2**22)
        self.pub_behavior = rospy.Publisher(
            '/behavior', BehaviorStatus, queue_size=1)

        self.schedule = Scheduler(self.pub_behavior)

    def deactivate(self):
        for behavior in self.behaviors:
            msg = BehaviorStatus()
            msg.name = behavior
            msg.status = False
            self.pub_behavior.publish(msg)

    def command_cb(self, msg):
        
        self.deactivate()
        events = self.commands[msg.data]
        self.schedule.create_schedule(events)
        print(self.schedule.queue)



    def loop(self):
        while not rospy.is_shutdown(): 
            self.schedule.publish()
            rospy.sleep(0.1)  



if __name__ == '__main__':
    rospy.init_node('command')
    command = Command()
    command.loop()
    