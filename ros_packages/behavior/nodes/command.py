import rospy
from multiprocessing import Lock
from behavior.msg import BehaviorStatus
from std_msgs.msg import empty
from time import time

# event <=> behavior

class Scheduler:
     def __init__(self):
        self.queue = []
        self.queue_mutex = Lock()
        current_time=time
        


