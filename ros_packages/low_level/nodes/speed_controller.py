#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from multiprocessing import Lock
import numpy as np
import datetime


class SpeedController:

    def __init__(self):
        #self.x_vel_max = get_param()
        self.current_velocity = Twist()

        #Publisher
        self.gathered_vel_pub = rospy.Publisher('/target_vel', Twist, queue_size=1)
        
        #Subscriber
        self.linear_x_sub = rospy.Subscriber('/linear_x', Float32, self.linear_x_cb)
        self.linear_y_sub = rospy.Subscriber('/linear_y', Float32, self.linear_y_cb)
        self.linear_z_sub = rospy.Subscriber('/linear_z', Float32, self.linear_z_cb)
        self.angular_z_sub = rospy.Subscriber('/angular_z', Float32, self.angular_z_cb)

        self.hover_mode = rospy.Subscriber('/hover', Empty, self.hover_cb)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        #Variables
        self.gathered_vel_msg = Twist()
        self.gathered_vel_msg_mutex = Lock()
        self.hover = True
        self.odom_mutex = Lock()
    

        self.gains = {
                "linear_x":   {'Kp': 2, 'Kd': 0.0, 'Ki': 0.0},
                "linear_y":  {'Kp': 0.0, 'Kd': 0.0, 'Ki': 0.0}
                }
        self.pid_roll = PID(self.gains["linear_x"])
        self.pid_pitch = PID(self.gains["linear_y"])


    def odom_cb(self, msg):
        with self.odom_mutex:
            self.current_velocity = msg.twist.twist

    def hover_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg = Twist()
            self.hover = True
        
    def linear_x_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.x = msg.data  
            self.hover = False

    def linear_y_cb(self, msg):
        
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.y = msg.data 
            self.hover = False 
    def linear_z_cb(self, msg):
      
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.linear.z = msg.data 
            self.hover = False      

    def angular_z_cb(self, msg):
       
        with self.gathered_vel_msg_mutex:
            self.gathered_vel_msg.angular.z = msg.data
            self.hover = False

    def loop(self):
        while not rospy.is_shutdown():
        
            with self.gathered_vel_msg_mutex:
            
                if self.hover:
                    self.gathered_vel_pub.publish(Twist())
                else:
                    t_ros = rospy.Time.now()
                    t_datetime = datetime.datetime.utcfromtimestamp(t_ros.to_sec()) 
                    self.cmd_vel_msg_output = Twist()
                    #self.gathered_vel_pub.publish(self.gathered_vel_msg)
                    self.pid_roll.update(t_datetime, - self.current_velocity.linear.x + self.gathered_vel_msg.linear.x)
                
                    self.pid_pitch.update(t_datetime, - self.current_velocity.linear.y + self.gathered_vel_msg.linear.y)

                    self.cmd_vel_msg_output.linear.x = self.pid_roll.command
                    
                    self.cmd_vel_msg_output.linear.y = self.pid_pitch.command
                    self.cmd_vel_msg_output.linear.z = self.gathered_vel_msg.linear.z
                    self.cmd_vel_msg_output.angular.z = self.gathered_vel_msg.angular.z
                    
                    self.gathered_vel_pub.publish(self.cmd_vel_msg_output)
            rospy.sleep(0.1)

class PID:

    def __init__(self, gains):
        '''
        Builds a PID

        Arguments:
            gains (dict): with keys Kp, Kd, Ki
        '''
        self.gains = gains.copy()
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }
        if not ('Kp' in gains and 'Kd' in gains and 'Ki' in gains):
            raise RuntimeError('''Some keys are missing in the gains, '''
                               '''we expect Kp, Kd and Ki''')


    def reset(self):
        '''
        Reset the errors of the PID
        '''
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }

    def update(self,
               time: datetime.datetime,
               error: float):
        '''
        Update the error, its derivative and integral
        '''
        prev_error = self.errors['error']
        prev_time = self.errors['time']
        
        if prev_time is None:
            self.errors['error'], self.errors['time'] = error, time
            return

        self.errors['error'] = error
        self.errors['time'] = time
        dt = (time - prev_time).total_seconds()
        self.errors['i_error'] += dt * (prev_error + error)/2.0
        self.errors['d_error'] = (error - prev_error)/dt

    @property
    def command(self):
        
        return self.gains['Kp'] * self.errors['error'] + \
                self.gains['Kd'] * self.errors['d_error'] + \
                self.gains['Ki'] * self.errors['i_error']



def main():
    """Instantiate node and class."""
    # declare node
    rospy.init_node('speed_controller')
    # instantiate class
    speed_controller = SpeedController()
    speed_controller.loop()
   
 

if __name__ == '__main__':
    main()