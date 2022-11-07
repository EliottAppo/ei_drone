import rospy
from multiprocessing import Lock
from behavior.msg import BehaviorStatus 


class Behavior:
    def __init__(self,name):
        self.stat = False
        self.name = name
        self.stat_mutex= Lock()

        self.sub_behavior= rospy.Subscriber('/behavior',BehaviorStatus,self.behav_cb)


    def get_status(self):
        with self.stat_mutex :
            return self.stat

    def activate(self):
        with self.stat_mutex:
            self.stat=True
    
    def inactivate(self):
        with self.stat_mutex:
            self.stat=False
    
    def behav_cb(self,msg):
        if msg.name == self.name and msg.status == True:
            self.activate()
        elif msg.name == self.name and msg.status == False:
            self.inactivate()

