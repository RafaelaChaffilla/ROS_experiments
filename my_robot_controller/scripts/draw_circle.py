#!/usr/bin/env python3

# the line above is the interpreter line, i dont know what it means

import rospy
from geometry_msgs.msg import Twist
import numpy as np

if __name__ == '__main__':
    rospy.init_node('draw_circle')
    rospy.loginfo('Draw circle has started')

    # queue size is a buffer that says we are going to hold 10 messages so that if our system takes time to listen to the message, it has a buffer to read from
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        # linear x,y and z are the turtles coordinates
        msg.linear.x  = 1.2
        msg.linear.y  = np.random.rand()*2
        msg.angular.z = np.random.rand()*5

        # send the message
        pub.publish(msg)
        
        # publish command velocity
        rate.sleep()

