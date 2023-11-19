#!/usr/bin/env python3

import rospy
from turtlesim.msg      import Pose
from geometry_msgs.msg  import Twist
from turtlesim.srv      import SetPen
import numpy as np

init_x, init_y = None, None
init_theta = None
state = 'init'

# define when circle will stop being drawn
y_margin = 0.7

def call_set_pen(r,g,b,width, off):
    ''' All input arguments have to be uint8'''
    try:
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        response = set_pen(r, g, b, width, off)
        # this service doesn't give a response
        # rospy.loginfo(response)
    except rospy.ServiceException as err:
        rospy.logwarn(err)

def set_state(pose: Pose):
    '''
        returns the current state that the path should take
    '''
    if state == 'init' and pose.theta >= np.pi/2:
        rospy.loginfo('switching states: ' + state)
        call_set_pen(135, 35, 65, 4, 0)
        return 'circle'
    elif state == 'circle' and pose.y <= init_y - y_margin:
        rospy.loginfo('switching states: ' + state)
        call_set_pen(240, 89, 65, 4, 0)
        return 'down_line'
    elif state == 'down_line' and pose.x >= init_x:
        rospy.loginfo('switching states: ' + state)
        return 'tip'
    elif state == 'tip' and pose.theta >= -init_theta:
        rospy.loginfo('switching states: ' + state)
        call_set_pen(190, 49, 68, 4, 0)
        return 'up_line'
    elif state == 'up_line' and pose.y >= init_y - y_margin:
        rospy.loginfo('switching states: ' + state)
        call_set_pen(34, 9, 44, 4, 0)
        return 'circle'
    elif (state == 'circle'
          and pose.x >= init_x - 0.1 and pose.x <= init_x + 0.1
          and pose.y >= init_y - 0.1 and pose.y <= init_y + 0.1
          and pose.theta < 0):
        call_set_pen(34, 9, 44, 0, 3)
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
    if(pose.y <= init_y - 0.5):
        rospy.signal_shutdown('Hertz complete <3')
        # return
    else:
        rule = Twist()
        rule.linear.x = 1
        rule.angular.z = 0.2
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

    # block the code from happening while the service is not up
    rospy.wait_for_service("/turtle1/set_pen")

    # create pub before sub ? prolly so that the sub knows the pub to publish inside the callback
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    rospy.loginfo('Node has been started')

    rospy.spin()