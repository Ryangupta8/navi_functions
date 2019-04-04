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
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def mains():
    # Initialize
    client = actionlib.SimpleActionClient('approach_action',navi_service.msg.ApproachAction)
    rospy.loginfo("wait_server")
    client.wait_for_server()
    rospy.loginfo("requesting_action_server")
    goal = navi_service.msg.ApproachGoal()

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=rospy.Time.now()
    pose.pose.position.x= 0.5
    pose.pose.position.y= -0.2
    goal_yaw =-math.pi/6
    goal_yaw =0.0
    quat = quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)

    goal.target=pose

    gpose = geometry_msgs.msg.PoseStamped()
    gpose.header.frame_id='map'
    gpose.header.stamp=rospy.Time.now()
    gpose.pose.position.x= 0.5
    gpose.pose.position.y= -4.0
    goal.gaze_target=gpose

    client.send_goal(goal)
    # rospy.loginfo("start action")
    client.wait_for_result(rospy.Duration(20.0))
    # rospy.loginfo("result %s", result)
    result = client.get_result()
    print result
        
if __name__ == '__main__':
    rospy.init_node('approach_client')
    mains()
