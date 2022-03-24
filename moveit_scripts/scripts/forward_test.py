#!/usr/bin/env python3
import sys
import copy
import rospy
import math
import numpy as np
import time
import open3d as o3d
import cv2
import actionlib

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import Int8, MultiArrayDimension, MultiArrayLayout, Int32MultiArray, Float32MultiArray, Bool, Header
from sensor_msgs.msg import PointCloud2, PointField, JointState
import sensor_msgs.point_cloud2 as pc2
from iiwa_msgs.msg import JointPosition, Spline, SplineSegment, MoveAlongSplineAction, MoveToJointPositionAction, MoveToJointPositionGoal, MoveAlongSplineGoal
from iiwa_msgs.srv import SetPTPJointSpeedLimits, SetEndpointFrame, SetPTPCartesianSpeedLimits
from moveit_msgs.srv import *
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

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

import time


if __name__ == '__main__':

    rospy.init_node('forward_kin_test', anonymous=True)
    rospy.wait_for_service('/iiwa/compute_fk')
    moveit_fk = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

    fkln = ['iiwa_link_ee']
    joint_names = []
    joint_positions = []
    for i in range(7):
      joint_names.append('iiwa_joint_'+str(i + 1)) # your names may vary
      joint_positions.append(0.8) # try some arbitrary joint angle
    header = Header(0,rospy.Time.now(),"iiwa_link_0")
    rs = RobotState()
    rs.joint_state.name = joint_names
    rs.joint_state.position = joint_positions
    rospy.loginfo(["FK LOOKUP:", moveit_fk(header, fkln, rs)]) # Lookup the pose
