#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np

init_x, init_y = None, None
init_theta = None
state = 'init'

# define when circle will stop being drawn
y_margin = 0.7

def set_state(pose: Pose):
    '''
        returns the current state that the path should take
    '''
    if state == 'init' and pose.theta >= np.pi/2:
        rospy.loginfo('switching states: ' + state)
        return 'circle'
    elif state == 'circle' and pose.y <= init_y - y_margin:
        rospy.loginfo('switching states: ' + state)
        rospy.loginfo('pose.y: ' + str(pose.y))
        rospy.loginfo('init.y: ' + str(init_y))
        return 'down_line'
    elif state == 'down_line' and pose.x >= init_x:
        rospy.loginfo('switching states: ' + state)
        return 'tip'
    elif state == 'tip' and pose.theta >= -init_theta:
        rospy.loginfo('switching states: ' + state)
        return 'up_line'
    elif state == 'up_line' and pose.y >= init_y - y_margin:
        rospy.loginfo('switching states: ' + state)
        return 'circle'
    elif (state == 'circle'
          and pose.x >= init_x - 0.1 and pose.x <= init_x + 0.1
          and pose.y >= init_y - 0.1 and pose.y <= init_y + 0.1
          and pose.theta < 0):
        return 'stop'
    else:
        return state

def init_travel(pose: Pose):
    global init_x
    global init_y
    if(init_x is None):
        init_x = pose.x 
        init_y = pose.y
    rule = Twist()
    rule.angular.z = 1
    return rule

def circle(pose: Pose): 
    rule = Twist()
    rule.linear.x = 1
    rule.angular.z = 1
    return rule

def downline(pose: Pose):
    rule = Twist()
    rule.linear.x = 1
    return rule

def tip(pose: Pose):
    global init_theta
    if(init_theta is None):
        init_theta = pose.theta
    rule = Twist()
    rule.angular.z = 1
    return rule

def stop(pose: Pose):
    rule = Twist()
    return rule

state_functions = {
    'init': init_travel,
    'circle': circle,
    'down_line': downline,
    'up_line': downline,
    'tip': tip,
    'stop': stop    
}

def pose_callback(msg: Pose):
    global state
    state = set_state(msg)
    cmd = state_functions[state](msg)
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('turtle_controller')

    # create pub before sub ? prolly so that the sub knows the pub to publish inside the callback
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    rospy.loginfo('Node has been started')

    rospy.spin()