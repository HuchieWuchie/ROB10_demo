#!/usr/bin/env python
import sys

import rospy
import moveit_commander
import moveit_msgs
from moveit_msgs.srv import GetPositionIK

from rob9.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from rob9.srv import moveitPlanToNamedSrv, moveitPlanToNamedSrvResponse
from rob9.srv import moveitMoveToPoseSrv, moveitMoveToPoseSrvResponse
from rob9.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from rob9.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse
from rob9.srv import moveitPlanToPoseSrv, moveitPlanToPoseSrvResponse

def moveToPose(req):

    print("Moving robot to cartesian pose goal: ", req.pose)
    move_group.set_pose_target(req.pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitMoveToPoseSrvResponse()
    resp.success.data = True

    return resp

def planToNamed(req):

    print("Computing plan to named position: ", req.name.data)

    get_start_state_client = rospy.ServiceProxy(baseServiceName + "getRobotState", moveitRobotStateSrv)
    response = get_start_state_client.call()
    move_group.set_start_state(response.state)
    move_group.set_named_target(req.name.data)
    plan = move_group.plan()

    resp = moveitPlanToNamedSrvResponse()
    resp.plan = plan

    return resp

def planToPose(req):

    print("Computing plan to given pose: ", req.pose)

    goal_pose = req.pose
    start_state = getCurrentState(0)

    goal_pose_msg = geometry_msgs.msg.PoseStamped()
    goal_pose_msg.header.frame_id = "world"
    goal_pose_msg.header.stamp = rospy.Time.now()
    goal_pose_msg.pose = goal_pose

    ik_request_msg = moveit_msgs.msg.PositionIKRequest()
    ik_request_msg.group_name = "manipulator"
    ik_request_msg.robot_state = start_state
    ik_request_msg.avoid_collisions = True #False
    ik_request_msg.pose_stamped = goal_pose_msg
    ik_request_msg.timeout = rospy.Duration(1.0) #
    ik_request_msg.attempts = 10

    rospy.wait_for_service('compute_ik')
    ik_calculator = rospy.ServiceProxy("compute_ik", GetPositionIK)

    goal_state = calculate_ik(request_msg)

    plan = 0

    if goal_state.error_code.val == 1:

        joint_states_at_goal = list(goal_state.solution.joint_state.position)
        joint_values_at_goal = copy.deepcopy(joint_states_at_goal[2:9])
        move_group.set_start_state(start_state)
        move_group.set_joint_value_target(joint_values_at_waypoint)
        plan = move_group.plan()

    resp = moveitPlanToPoseSrvResponse()
    resp.plan = plan

    return resp


def moveToNamed(req):

    print("Moving robot to named position: ", req.name.data)

    move_group.set_named_target(req.name.data)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitMoveToNamedSrvResponse()
    resp.success.data = True

    return resp

def execute(req):

    move_group.execute(req.trajectory, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitExecuteSrvResponse()
    resp.success.data = True

    return resp

def getCurrentState(req):

    return robot.get_current_state()

if __name__ == '__main__':

    baseServiceName = "/rob9/moveit/"

    rospy.init_node('moveit_service', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    moveToNameService = rospy.Service(baseServiceName + "move_to_named", moveitMoveToNamedSrv, moveToNamed)
    planToNameService = rospy.Service(baseServiceName + "plan_to_named", moveitPlanToNamedSrv, planToNamed)
    planToNameService = rospy.Service(baseServiceName + "plan_to_pose", moveitPlanToPoseSrv, planToPose)
    moveToPoseService = rospy.Service(baseServiceName + "move_to_pose", moveitMoveToPoseSrv, moveToPose)
    executeService = rospy.Service(baseServiceName + "execute", moveitExecuteSrv, execute)
    robotStateService = rospy.Service(baseServiceName + "getRobotState", moveitRobotStateSrv, getCurrentState)

    rospy.spin()
