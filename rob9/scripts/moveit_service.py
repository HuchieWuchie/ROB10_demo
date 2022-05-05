#!/usr/bin/env python
import sys

import rospy
import moveit_commander
import moveit_msgs

from rob9.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from rob9.srv import moveitPlanToNamedSrv, moveitPlanToNamedSrvResponse
from rob9.srv import moveitMoveToPoseSrv, moveitMoveToPoseSrvResponse
from rob9.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from rob9.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse

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

    goal_pose = 0
    start_state = 0

    state = get_ik(goal_pose, start_state)

    if state_at_waypoint.error_code.val == 1:

        joint_states_at_goal = list(state.solution.joint_state.position)
        joint_values_at_goal = copy.deepcopy(joint_states_at_goal[2:9])
        move_group.set_start_state(start_state)
        move_group.set_joint_value_target(joint_values_at_waypoint)
        plan = move_group.plan()


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
    moveToPoseService = rospy.Service(baseServiceName + "move_to_pose", moveitMoveToPoseSrv, moveToPose)
    executeService = rospy.Service(baseServiceName + "execute", moveitExecuteSrv, execute)
    robotStateService = rospy.Service(baseServiceName + "getRobotState", moveitRobotStateSrv, getCurrentState)

    rospy.spin()
