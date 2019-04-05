#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.msg import *
import math


class BaseSlideAction(object):

    def __init__(self, name):
        # Init actionserver

        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        self._as = actionlib.SimpleActionServer(self._action_name, navi_service.msg.BaseSlideAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("base_slide")

        tw = geometry_msgs.msg.Twist()
        if goal.cmd ==1:
            tw.linear.x=0.25
        elif goal.cmd ==2:
            tw.linear.x=-0.25
        
        vel_pub.publish(tw)
        # rospy.sleep(0.5)
        # self.body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.9,"wrist_roll_joint":-1.2, "wrist_flex_joint":0.2})
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('baseslide_action')
    rospy.loginfo("Initializing baseslide server...")
    server = BaseSlideAction('baseslide_action')
    rospy.loginfo("baseslide_action server created")
    rospy.spin()
