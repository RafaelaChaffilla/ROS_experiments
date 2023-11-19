#!/usr/bin/env python3
import rospy

# runs code only inside the main function
if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo("Test node is on")

    rate = rospy.Rate(3)

    # if node has received shutdown request, rospy.is_Shutdown is true
    # so this happens while the node isn't shuted down
    while not rospy.is_shutdown():
        rospy.loginfo('hallo')
        rate.sleep()
    