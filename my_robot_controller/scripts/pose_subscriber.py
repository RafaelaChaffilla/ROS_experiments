#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

# using the msg: Pose helps with code reading but mostly autocompletion
def pose_callback(msg: Pose):
    rospy.loginfo('i hear somethisn whhhhhhhhhhhhhhhhhhhhhhaaaaaaaaaaaaaaaaaaat')
    rospy.loginfo(msg)
    rospy.logwarn('(' + str(msg.x) + ',' + str(msg.y) + ')')

if __name__ == '__main__':
    rospy.init_node('turtle_pose_sub')

    # callback is the function to be executed when we receive a message in the topic
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    rospy.loginfo('listener node started')
    
    # is kinda the infinite loop that allows the node to run and the sub to listen always
    # it's more of a passive thing that allows the code to run without having to do anything
    # usually is the last line of the code
    rospy.spin()