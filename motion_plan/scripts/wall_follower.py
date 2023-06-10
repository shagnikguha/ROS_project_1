#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

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
    msg = Twist()
    linear_x = 0
    angular_z = 0
    if regions['right'] < 2  and regions['fright'] < 2:
        linear_x = 0
        angular_z = -0.6
    elif regions['left'] < 2 and regions['fleft'] < 2:
        linear_x = 0
        angular_z = 0.6
    elif regions['fleft'] > 2 and regions['front'] > 2 and regions['fright'] > 2:
        linear_x = 0.6
        angular_z = 0    #change angule make bigger stuck at wall

    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)




def main():
    global pub
    
    rospy.init_node('reading_laser')    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)    
    rospy.spin()

if __name__ == '__main__':
    main()




