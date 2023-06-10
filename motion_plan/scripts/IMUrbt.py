#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist  
pub = None
rospy.init_node('imu_odometry')   #used as last time is giving error otherwise
last_Time = rospy.Time.now()
lastPosition = Vector3(0, 0, 0)
lastVelocity = Vector3(0, 0, 0)
lastOrientation = Quaternion(0, 0, 0, 1)
lastAngular_velocity = Vector3(0, 0, 0)


def imu_clbk(data):
    global last_Time, lastPosition, lastVelocity, lastOrientation, lastAngular_velocity

    currentTime = rospy.Time.now()
    dt = (currentTime - last_Time).to_sec()
    
    linearAcceleration = data.linear_acceleration
    angularVelocity = data.angular_velocity
    orientation = data.orientation
    
    velocity = Vector3(lastVelocity.x + linearAcceleration.x * dt, lastVelocity.y + linearAcceleration.y * dt, lastVelocity.z + linearAcceleration.z * dt)
    position = Vector3(lastPosition.x + velocity.x * dt, lastPosition.y + velocity.y * dt, lastPosition.z + velocity.z * dt)
    
    #basic odom cmd from rospy web
    odom = Odometry()
    odom.header.stamp = currentTime
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    #posting Pose values
    odom.pose.pose.orientation = orientation
    odom.pose.pose.position.x = position.x
    odom.pose.pose.position.y = position.y
    odom.pose.pose.position.z = position.z
    #posting Twist values
    odom.twist.twist.angular = angularVelocity
    odom.twist.twist.linear.x = velocity.x
    odom.twist.twist.linear.y = velocity.y
    odom.twist.twist.linear.z = velocity.z
    #publishing 
    pub.publish(odom)
    
    
    lastPosition = position
    lastVelocity = velocity
    lastOrientation = orientation
    lastAngular_velocity = angularVelocity
    
def main():
    global pub
    rate = rospy.Rate(10) 
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    sub = rospy.Subscriber('/imu', Imu, imu_clbk)  
    rospy.spin()

if __name__ == '__main__':
    main()


'''v = v0 + a * t
d = d0 + d * t
'''