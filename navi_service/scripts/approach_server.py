#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped
from hsrb_interface import Robot, exceptions, geometry
from navi_service.msg import *
from sensor_msgs.msg import JointState
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math


class ApproachServer(object):

    def __init__(self, name, robot):
        # Init actionserver
        self._action_name = name
        self.robot = robot
        self.robot_pose=np.zeros((3,1))
        self.target_pose = Pose()
        self.IsActive = True
        self.IsGoal= False

        # Preparation to use robot functions
        while not rospy.is_shutdown():
            try:
                self.body = self.robot.try_get('whole_body')
                self.gripper = self.robot.try_get('gripper')
                self.omni_base = self.robot.try_get('omni_base')
                break
            except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

        self.vel_pub = rospy.Publisher("objet_tracker/particle_filter",Image,queue_size=10)

        robot_pose_topic='global_pose'
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.robot_pose_Cb)

        self._as = actionlib.SimpleActionServer(self._action_name, navi_service.msg.ApproachAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):

        self.body.move_to_neutral()
        rospy.sleep(3)
        self.open_gripper()
        rospy.loginfo("base pose")
        self.body.move_to_joint_positions({"arm_lift_joint":0.3, "arm_flex_joint":-0.6,"arm_roll_joint":-1.57,"wrist_roll_joint":-0.7,"wrist_flex_joint":-0.5})
        rospy.sleep(3)

        self._as.set_succeeded()

    def robot_pose_Cb(self, msg):
        #self_robot_pose[0] = robot_position_x
        #self_robot_pose[1] = robot_position_y
        #self_robot_pose[2] = robot_theta_yaw

        self.robot_pose[0]=msg.pose.position.x
        self.robot_pose[1]=msg.pose.position.y
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)
        self.robot_pose[2]=yaw
    def send_vel_cmmand(self):
        if IsActive:
            if IsGoal==False:
                

 

if __name__ == '__main__':
    robot = Robot()
    # rospy.loginfo("Initializing approach server...")
    server = ApproachServer('approach_action', robot)
    rospy.loginfo("approach_action server created")
    rospy.spin()
