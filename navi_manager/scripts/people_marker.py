#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8
import std_msgs.msg
import rospy
import math

# topic = 'human_target'
topic_array = 'human_poses_boxes'
# publisher = rospy.Publisher(topic, Marker,queue_size=10)
# array_publisher = rospy.Publisher(topic_array, MarkerArray,queue_size=10)
array_publisher = rospy.Publisher(topic_array, PoseArray,queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()
posesrArray = PoseArray()

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

   # del markerArray.markers[:]
   del posesrArray.poses[:]

   pose_position = Point()
   pose_orientation = Quaternion()

   pose_position.x=1.0
   pose_position.y=1.0
   pose_position.z=0.5

   pose_orientation.x=0
   pose_orientation.y=0
   pose_orientation.z=0
   pose_orientation.w=1.0
   Human_pose = Pose()

   Human_pose.position=pose_position
   Human_pose.orientation=pose_orientation
   posesrArray.poses.append(Human_pose)

   pose_position.x=5.0
   pose_position.y=3.0
   pose_position.z=0.5

   pose_orientation.x=0
   pose_orientation.y=0
   pose_orientation.z=0
   pose_orientation.w=1.0
   Human_pose2 = Pose()

   Human_pose2.position=pose_position
   Human_pose2.orientation=pose_orientation
   posesrArray.poses.append(Human_pose2)


   array_publisher.Publish(pose)
   #moving human
   #marker.pose.position.x = 2.2+0.0015*count
   #marker.pose.position.y = -0.3+0.2*math.sin(count/100.0)
   #marker.pose.position.z = 1

   # marker.pose.position.y = 2.7+0.2*math.sin(count/100.0)
   # marker.pose.position.y = +0.2*math.sin(count / 40.0) 
   
   # marker.pose.position.x = 3.0
   # marker.pose.position.y = 0.5
   # marker.pose.position.z = 1

   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   # if(count > MARKERS_MAX):
   #     markerArray.markers.pop(0)
   # Renumber the marker IDs
   # id = 0
   # for m in markerArray.markers:
       # m.id = id
       # id += 1

   # Publish the MarkerArray
   # publisher.publish(marker)
   # array_publisher.publish(markerArray)
   # pub.publish(int_msg)

   count += 1

   rospy.sleep(0.1)
