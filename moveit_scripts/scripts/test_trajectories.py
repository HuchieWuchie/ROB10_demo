#!/usr/bin/env python3
import sys
import copy
import rospy
import math
import numpy as np
import time
import open3d as o3d
import cv2

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion
import std_msgs.msg
from std_msgs.msg import Int8, MultiArrayDimension, MultiArrayLayout, Int32MultiArray, Float32MultiArray, Bool, Header
from sensor_msgs.msg import PointCloud2, PointField, JointState
import sensor_msgs.point_cloud2 as pc2
from iiwa_msgs.msg import JointPosition, Spline, SplineSegment, MoveAlongSplineActionGoal

import rob9Utils.transformations as transform
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp
import rob9Utils.moveit as moveit
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
from rob9Utils.visualize import visualizeGrasps6DOF

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *
from rob9.srv import graspGroupSrv, graspGroupSrvResponse
from rob9.srv import moveitMoveToPoseSrv, moveitMoveToPoseSrvResponse

import time

def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory_start = moveit.getCurrentState()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

def pub_joint_command(plan):
    print("pub_joint_command")
    #print(plan)
    #print(len(plan.joint_trajectory.points))

    ts = time.time() * 1000
    for joint_positions in plan.joint_trajectory.points:
        joint_goal = JointPosition()
        joint_goal.header.frame_id = ""
        joint_goal.header.stamp = rospy.Time.now()
        joint_goal.position.a1 = joint_positions.positions[0]
        joint_goal.position.a2 = joint_positions.positions[1]
        joint_goal.position.a3 = joint_positions.positions[2]
        joint_goal.position.a4 = joint_positions.positions[3]
        joint_goal.position.a5 = joint_positions.positions[4]
        joint_goal.position.a6 = joint_positions.positions[5]
        joint_goal.position.a7 = joint_positions.positions[6]
        print("Done creating message")
        pub_iiwa.publish(joint_goal)

    te = time.time() * 1000
    print("Send joint trajectory in ", te - ts, " ms")

def callbackJointState(msg):
    global robot_is_moving

    positive_velocity = False
    for vel in msg.velocity:
        if round(vel, 1) != 0:
            positive_velocity = True

    if positive_velocity:
        robot_is_moving = True
    else:
        robot_is_moving = False

if __name__ == '__main__':

    print("Init")
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('/iiwa/joint_states', JointState, callbackJointState)
    pub_iiwa = rospy.Publisher('iiwa/command/JointPosition', JointPosition, queue_size=10 )
    pub_spline = rospy.Publisher('iiwa/action/move_along_goal/goal', MoveAlongSplineActionGoal, queue_size=1 )
    display_trajectory_publisher = rospy.Publisher('iiwa/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    #moveit.moveToNamed("ready")

    right_point = Point()
    right_point.x = 0.613034243309
    right_point.y = 0.313086143769
    right_point.z = 0.25 #0.186170889279

    right_quat = Quaternion()
    #quat = np.array([-0.332310264419,
    #                0.740404188633,
    #                0.58174091977,
    #                0.0543046519347])

    quat = np.array([-0.7070500545729,
                    0.0167979008382,
                    -0.0166933141001,
                    0.706766843796])
    quat = quat / np.sqrt(np.sum(quat**2))
    right_quat.x = quat[0]
    right_quat.y = quat[1]
    right_quat.z = quat[2]
    right_quat.w = quat[3]


    right_quat.x = 0
    right_quat.y = 0
    right_quat.z = 0
    right_quat.w = -1

    right_pose = Pose()
    right_pose.position = right_point
    right_pose.orientation = right_quat

    right_header = Header()
    right_header.seq = 0
    right_header.stamp = rospy.Time.now()
    right_header.frame_id = ""

    right_posestamped = PoseStamped()
    right_posestamped.header = right_header
    right_posestamped.pose = right_pose

    rightSplineAction = MoveAlongSplineActionGoal()
    right_spline_segment = SplineSegment()
    right_spline_segment.point = right_posestamped

    right_spline_segment2 = SplineSegment()
    right_spline_segment2.point = right_posestamped
    right_spline_segment2.point.pose.position.x += -0.05

    right_spline = Spline()
    right_spline.segments.append(right_spline_segment)
    right_spline.segments.append(right_spline_segment2)

    rightSplineAction.goal.spline = right_spline
    pub_spline.publish(rightSplineAction)
    print(rightSplineAction)
    exit()


    left_point = Point()
    left_point.x = -0.142646580148
    left_point.y = 0.507771203615
    left_point.z = 0.452486649806

    left_quat = Quaternion()
    quat = np.array([-0.203572167467,
                    0.71279925108,
                    0.6380314375,
                    0.208306207341])
    quat = quat / np.sqrt(np.sum(quat**2))
    left_quat.x = quat[0]
    left_quat.y = quat[1]
    left_quat.z = quat[2]
    left_quat.w = quat[3]


    left_pose = Pose()
    left_pose.position = right_point
    left_pose.orientation = right_quat

    left_header = Header()
    left_header.seq = 1
    left_header.stamp = rospy.Time.now()
    left_header.frame_id = "world"

    left_posestamped = PoseStamped()
    left_posestamped.header = right_header
    left_posestamped.pose = right_pose

    path_msg = nav_msgs.msg.Path()
    path_msg.header.frame_id = "world"
    path_msg.header.stamp = rospy.Time.now()
    path_msg.poses.append(right_posestamped)
    path_msg.poses.append(left_posestamped)
    #get_trajectories = rospy.ServiceProxy('/rob9/moveit/move_to_pose', moveitMoveToPoseSrv)
    #print(right_pose)
    #print(transform.transformToFrame(right_posestamped, newFrame = "world", currentFrame = "iiwa_link_0").pose)
    #resp_trajectories = get_trajectories(transform.transformToFrame(right_posestamped, newFrame = "world", currentFrame = "iiwa_link_0").pose)




    """
    print("Calling the trajectory service")
    rospy.wait_for_service('iiwa/get_trajectories')
    get_trajectories = rospy.ServiceProxy('iiwa/get_trajectories', GetTrajectories)
    resp_trajectories = get_trajectories(path_msg)
    print("I have received a trajectory server response ")
    print(resp_trajectories)

    plans = []
    goal_poses = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        plans.append(resp_trajectories.trajectories.trajectories[i])

    for i in range(3):
        send_trajectory_to_rviz(plans[i])
        #print(type(plans[i]))
        pub_joint_command(plans[i]) # outcommented by Albert Wed 23 March 09:06
        #moveit.execute(plans[i]) # incommented by Albert Wed 23 March 09:06
        while robot_is_moving == True:
            print("robot_is_moving: ", robot_is_moving)
            rospy.sleep(0.1)
        if i == 1:
            gripper_pub.publish(close_gripper_msg) # incommented by Albert Wed 23 March 09:06
            rospy.sleep(1) # incommented by Albert Wed 23 March 09:06
            print("I have grasped!")
        input("Press Enter when you are ready to move the robot back to the ready pose") # outcommented by Albert Wed 23 March 09:06

    while robot_is_moving == True:
        rospy.sleep(0.1)
    moveit.moveToNamed("ready")
    while robot_is_moving == True:
        rospy.sleep(0.1)
    moveit.moveToNamed("handover")
    while robot_is_moving == True:
        rospy.sleep(0.1)
    #input("Press Enter when you are ready to move the robot back to the ready pose")

    #moveit.moveToNamed("ready")
    gripper_pub.publish(open_gripper_msg)
    """
    print("Here")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
