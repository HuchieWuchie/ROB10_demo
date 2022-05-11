#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import String, Bool

from rob9.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from rob9.srv import moveitPlanToNamedSrv, moveitPlanToNamedSrvResponse
from rob9.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from rob9.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse
from rob9.srv import moveitPlanToPoseSrv, moveitPlanToPoseSrvResponse
from rob9.srv import moveitPlanFromPoseToPoseSrv, moveitPlanFromPoseToPoseSrvResponse

def moveToNamed(name):

    if not len(name):
        print("ERROR: Specify a named pose")
        return 0

    rospy.wait_for_service("/rob9/moveit/move_to_named")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/move_to_named", moveitMoveToNamedSrv)

    msg = moveitMoveToNamedSrv()
    msg.data = name

    success = tf2Service(msg)

    return success

def execute(plan):

    rospy.wait_for_service("/rob9/moveit/execute")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/execute", moveitExecuteSrv)

    msg = moveitExecuteSrv()
    msg = plan

    success = tf2Service(msg)
    print(success)

    return success

def planToNamed(name):

    rospy.wait_for_service("/rob9/moveit/plan_to_named")
    service = rospy.ServiceProxy("/rob9/moveit/plan_to_named", moveitPlanToNamedSrv)

    msg = moveitPlanToNamedSrv()
    msg.data = name

    response = service(msg)
    return response.plan

def planFromPoseToPose(start_pose, goal_pose):

    rospy.wait_for_service("/rob9/moveit/plan_from_pose_to_pose")
    service = rospy.ServiceProxy("/rob9/moveit/plan_from_pose_to_pose", moveitPlanFromPoseToPoseSrv)

    response = service(start_pose, goal_pose)
    return response.success, response.plan

def planToPose(pose):

    rospy.wait_for_service("/rob9/moveit/plan_to_pose")
    service = rospy.ServiceProxy("/rob9/moveit/plan_to_pose", moveitPlanToPoseSrv)

    response = service(pose)
    return response.success, response.plan

def getCurrentState():

    rospy.wait_for_service("/rob9/moveit/getRobotState")
    tf2Service = rospy.ServiceProxy("/rob9/moveit/getRobotState", moveitRobotStateSrv)

    msg = moveitRobotStateSrv()
    msg.data = True

    state = tf2Service(msg).state

    return state
