#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

diff = 0

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp        #proportional constant
        self.ki = ki        #integral constant
        self.kd = kd        #differential constant
        self.fw_dist = 0    #desired distance from wall
        self.c_dist = 0     #current distance from wall
        self.Error = 0      #current error
        self.kiError = 0    #error sum
        self.prevError = 0  #previous error
    
    def control(self, w_dist, c_dist):
        self.w_dist = w_dist
        self.c_dist = c_dist
        self.Error = self.w_dist - self.c_dist                      #error calculation
        self.Proportional = self.kp * self.Error                    #proportional out 
        self.kiError = self.kiError + self.Error                    #total error calculation
        self.Integral = self.ki * self.kiError                      #integral out    
        self.Differential = self.kd * (self.Error*self.prevError)   #differential out
        self.prevError = self.Error

        self.Total_out =  self.Proportional+self.Integral+self.Differential

        return self.Total_out
    

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    
    take_action(regions)

def take_action(regions):
    global diff
    obj = PID(0.5, 0.005, 0.2)

    msg = Twist()
    
    w_dist = regions['left']
    c_dist = regions['right']

    Total_out = obj.control(w_dist, c_dist)

    f_dist = regions['front']           #current forward dist. 2 is max limit

    if(f_dist<2):
        diff = obj.control(2, f_dist)
    else:
        diff = 0
    linear_x = 0.8 -diff
    angular_z = Total_out
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('wall_centering_PID')   
    rate = rospy.Rate(10) 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)    
    rospy.spin()

if __name__ == '__main__':
    main()


