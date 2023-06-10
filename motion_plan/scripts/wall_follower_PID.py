#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None


kp = 0.5        #proportional constant
ki = 0.005      #integral constant
kd = 0.2        #differential constant
w_dist = 0      #desired distance from wall
c_dist = 0      #current distance from wall
Error = 0       #current error
kiError = 0     #error sum
prevError = 0   #previous error

 
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
    global w_dist, c_dist, Error, kiError, prevError

    msg = Twist()
    
    w_dist = regions['left']
    c_dist = regions['right']
    Error = w_dist - c_dist         #error calculation
    Proportional = kp * Error       #proportional out 

    kiError = kiError + Error       #total error calculation
    Integral = ki * kiError         #integral out    

    Differential = kd * (Error*prevError)   #differential out

    prevError = Error

    Total_out =  Proportional+Integral+Differential

    
    linear_x = 0.3
    angular_z = Total_out
    msg.linear.x = linear_x
    msg.angular.z = -angular_z
    pub.publish(msg)




def main():
    global pub
    rospy.init_node('wall_centering')   
    rate = rospy.Rate(10) 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)    
    rospy.spin()

if __name__ == '__main__':
    main()
