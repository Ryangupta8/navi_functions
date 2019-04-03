#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
import actionlib
from geometry_msgs.msg import *
from navi_service.msg import *


def mains():
    # Initialize
    client = actionlib.SimpleActionClient('obscheck_action',navi_service.msg.ObsCheckerAction)
    rospy.loginfo("wait_server")
    client.wait_for_server()
    rospy.loginfo("requesting_action_server")
    goal = navi_service.msg.ObsCheckerGoal()
    pose = geometry_msgs.msg.PointStamped()
    pose.header.frame_id='map'
    pose.header.stamp=rospy.Time.now()
    pose.point.x= 0.8
    pose.point.y= 0.0
    goal.pose =pose
    client.send_goal(goal)
    result= client.wait_for_result()
    print result
    # rospy.loginfo("start action")
        
if __name__ == '__main__':
    rospy.init_node('obschecker_client')
    mains()
