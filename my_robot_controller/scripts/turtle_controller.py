#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def pose_callback(msg: Pose):
    cmd = Twist()
    if msg.x > 9 or msg.x < 2 or msg.y > 9 or msg.y < 2:
        cmd.linear.x = 1
        cmd.angular.z = 1.4
    else:
        cmd.linear.x = 5
        cmd.angular.z = 0
    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    rospy.loginfo("Node has been started") 

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    

    rospy.spin()