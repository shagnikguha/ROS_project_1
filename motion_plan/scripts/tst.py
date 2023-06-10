#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

rospy.init_node('map_publisher')

# Set up the ROS publisher for the map
map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

# Set the resolution of the map
resolution = 0.1

# Set the size of the map (in meters)
map_size = 50

# Initialize the map
map_data = np.zeros((int(map_size / resolution), int(map_size / resolution)), dtype=np.int8)

def scan_callback(scan_msg):
    global map_data

    # Get the range data from the laser scan message
    ranges = np.array(scan_msg.ranges)

    # Convert the range data to Cartesian coordinates
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Convert the Cartesian coordinates to map coordinates
    map_x = (x / resolution + map_size / 2).astype(int)
    map_y = (y / resolution + map_size / 2).astype(int)

    # Update the map with the new data
    for i in range(len(map_x)):
        if map_x[i] >= 0 and map_x[i] < map_data.shape[0] and map_y[i] >= 0 and map_y[i] < map_data.shape[1]:
            map_data[map_x[i], map_y[i]] = 100

    # Create an OccupancyGrid message and publish it
    map_msg = OccupancyGrid()
    map_msg.header = scan_msg.header
    map_msg.info.resolution = resolution
    map_msg.info.width = map_data.shape[0]
    map_msg.info.height = map_data.shape[1]
    map_msg.info.origin.position.x = -map_size / 2
    map_msg.info.origin.position.y = -map_size / 2
    map_msg.info.origin.orientation.w = 1
    map_msg.data = map_data.flatten()
    map_pub.publish(map_msg)

rospy.Subscriber('/scan', LaserScan, scan_callback)

rospy.spin()
