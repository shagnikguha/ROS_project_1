#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from laser_geometry import LaserProjection
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist  
pub = None
rospy.init_node('SLAM')   
laser_projector = LaserProjection()

#intigrating IMU data into Odometry data
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
    '''
    v = v0 + a * t
    d = d0 + d * t
    '''
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


#getting data from LaserScan. Getting data of large area instead of certain angles only
def laser_clbk(msg):
    global laser_projector
    pointcloud2_msg = laser_projector.projectLaser(msg)
    

def main():
    rate = rospy.Rate(10)
    laser_sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, laser_clbk)
    imu_sub = rospy.Subscriber('/imu', Imu, imu_clbk)
    #odom_sub = rospy.Subscriber('/odom', Odometry, odom_clbk)
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)

if __name__=='__main__':
    main()



