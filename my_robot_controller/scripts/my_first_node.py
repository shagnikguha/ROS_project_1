#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node("test_node")    #name of node is test_node
    rospy.loginfo("test node started")  #output

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():  #will run until node is terminated with ^C in terminal
        rospy.loginfo("Hello")
        rate.sleep() #sleep with data from rate, i.e 10hz, i.e 0.1sec



