#!/usr/bin/env python3
import sys
import copy
import rospy
import math
import numpy as np
import time
import open3d as o3d
import cv2
#import actionlib

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import Int8, MultiArrayDimension, MultiArrayLayout, Int32MultiArray, Float32MultiArray, Bool, Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import rob9Utils.transformations as transform
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp
import rob9Utils.moveit as moveit
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
from orientationService.client import OrientationClient
from locationService.client import LocationClient
from rob9Utils.visualize import visualizeGrasps6DOF, visualizeMasksInRGB
import rob9Utils.iiwa

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *
from rob9.srv import graspGroupSrv, graspGroupSrvResponse

import time


def associateGraspAffordance(graspData, objects, masks, cloud, cloud_uv, demo = False):

    graspMsg = graspData.toGraspGroupMsg()

    objectMsg = Int32MultiArray()
    objectMsg.data = objects.tolist()

    intToLabel = {0: 'class', 1: 'height', 2: 'width'}
    maskMsg = Int32MultiArray()

    masks = np.reshape(masks, (-1, masks.shape[2], masks.shape[3]))

    # constructing mask message
    for i in range(3):
        dimMsg = MultiArrayDimension()
        dimMsg.label = intToLabel[i]
        stride = 1
        for j in range(3-i):
            stride = stride * masks.shape[i+j]
        dimMsg.stride = stride
        dimMsg.size = masks.shape[i]
        maskMsg.layout.dim.append(dimMsg)
    maskMsg.data = masks.flatten().astype(int).tolist()

    demoMsg = Bool()
    demoMsg.data = demo

    uvDim1 = MultiArrayDimension()
    uvDim1.label = "length"
    uvDim1.size = int(cloud_uv.shape[0] * cloud_uv.shape[1])
    uvDim1.stride = cloud_uv.shape[0]

    uvDim2 = MultiArrayDimension()
    uvDim2.label = "pair"
    uvDim2.size = cloud_uv.shape[1]
    uvDim2.stride = cloud_uv.shape[1]

    uvLayout = MultiArrayLayout()
    uvLayout.dim.append(uvDim1)
    uvLayout.dim.append(uvDim2)

    uvMsg = Float32MultiArray()
    uvMsg.data = cloud_uv.flatten().tolist()
    uvMsg.layout = uvLayout

    FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "ptu_camera_color_optical_frame"
    cloudMsg = pc2.create_cloud(header, FIELDS_XYZ, cloud)

    rospy.wait_for_service('grasp_affordance_association/associate')
    get_grasps_service = rospy.ServiceProxy('grasp_affordance_association/associate', graspGroupSrv)
    response = get_grasps_service(demoMsg, graspMsg, objectMsg, maskMsg, cloudMsg, uvMsg)

    return GraspGroup().fromGraspGroupSrv(response)

def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory_start = moveit.getCurrentState()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def callback(msg):
    global resp_trajectories, grasps_affordance, pcd, masks, cloud_uv, bboxs

    id = msg.data[0]
    requested_affordance_id = msg.data[1]

    affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]

    # Select grasps of objects with correct affordance
    graspObj = GraspGroup(grasps = copy.deepcopy(grasps_affordance.getgraspsByTool(id = id)))
    grasps_affordance_tool = GraspGroup(grasps = copy.deepcopy(graspObj.getgraspsByAffordanceLabel(label = requested_affordance_id)))

	# Compute waypoints for each grasp, could be a for loop instead with a break statement
    grasp_waypoints_path = computeWaypoints(grasps_affordance_tool, offset = 0.1)

    print("Calling the trajectory service")
    rospy.wait_for_service('iiwa/get_trajectories')
    get_trajectories = rospy.ServiceProxy('iiwa/get_trajectories', GetTrajectories)
    resp_trajectories = get_trajectories(grasp_waypoints_path)
    print("I have received a trajectory server response ")

    id_list_duplicates = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        id_list_duplicates.append(resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id)
    id_list = list(dict.fromkeys(id_list_duplicates))
    print("id_list " + str(id_list))

    id = str(id)
    plans = []
    goal_poses = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        if resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id == id:
            plans.append(resp_trajectories.trajectories.trajectories[i])

    for i in range(len(resp_trajectories.trajectories_poses.poses)):
        if resp_trajectories.trajectories_poses.poses[i].header.frame_id == id:
            goal_poses.append(resp_trajectories.trajectories_poses.poses[i])

	# Construct poseStamped messages

    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header.frame_id = "world"
    waypoint_msg.header.stamp = rospy.Time.now()
    waypoint_msg.pose = goal_poses[0].pose
    pub_waypoint.publish(waypoint_msg) # Publish to RVIZ visualization

    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_msg.header.frame_id = "world"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose = goal_poses[1].pose
    pub_grasp.publish(goal_msg) # Publish to RVIZ visualization

    for i in range(3):
        send_trajectory_to_rviz(plans[i])
        rob9Utils.iiwa.execute_spline_trajectory(plans[i])
        if i == 1:
            gripper_pub.publish(close_gripper_msg) # incommented by Albert Wed 23 March 09:06
            rospy.sleep(1) # incommented by Albert Wed 23 March 09:06
            print("I have grasped!")
        #input("Press Enter when you are ready to move the robot back to the ready pose") # outcommented by Albert Wed 23 March 09:06
    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))

    # computing goal pose of object in camera frame and
    # current pose of object in camera frame
    ## 	  - Use ICP to get current goal orientation and position

    geometry = np.asanyarray(pcd.points)
    rotClient = OrientationClient()
    current_orientation, current_position, goal_orientation = rotClient.getOrientation(geometry, cloud_uv, masks[id], bboxs[id]) # we discard translation

    # transform current pose into world frame
    curr_rot_quat = transform.quaternionFromRotation(current_orientation)
    curr_pose = np.hstack((current_position.flatten(), curr_rot_quat))
    curr_pose_world = transform.transformToFrame(curr_pose, "world", "ptu_camera_color_optical_frame")

    ## Get goal position using linescan service

    goal_rot_quat = transform.quaternionFromRotation(goal_orientation)

    locClient = LocationClient()
    goal_location = locClient.getLocation()
    goal_location_giver = transform.transformToFrame(curr_pose, "giver", "world")
    goal_pose_giver = np.hstack((goal_location.flatten(), goal_rot_quat))

    goal_pose_world = transform.transformToFrame(goal_pose_giver, "world", "giver")

    grasp_pose_world = goal_msg

    # Compute the homegenous 4x4 transformation matrices

    world_grasp_T = poseStampedToMatrix(grasp_pose_world)
    world_centroid_T = poseStampedToMatrix(curr_pose_world)
    world_centroid_T_goal = poseStampedToMatrix(goal_pose_world)

    # Compute an end effector pose that properly orients the grasped tool

    grasp_world_T = np.linalg.inv(world_grasp_T)
    grasp_centroid_T = np.matmul(grasp_world_T, world_centroid_T)

    centroid_grasp_T = np.linalg.inv(grasp_centroid_T)
    world_grasp_T_goal = np.matmul(world_centroid_T_goal, centroid_grasp_T)
    goal_q = quaternion_from_matrix(world_grasp_T_goal)

    world_centroid_T_test = np.matmul(world_grasp_T, grasp_centroid_T)
    world_centroid_T_goal_test = np.matmul(world_grasp_T_goal, grasp_centroid_T)

    # Create poseStamped ros message

    ee_goal_msg = geometry_msgs.msg.PoseStamped()
    ee_goal_msg.header.frame_id = "world"
    ee_goal_msg.header.stamp = rospy.Time.now()

    ee_pose = Pose()
    ee_pose.position.x = world_grasp_T_goal[0,3]
    ee_pose.position.y = world_grasp_T_goal[1,3]
    ee_pose.position.z = world_grasp_T_goal[2,3]

    ee_pose.orientatoin.x = goal_q[0]
    ee_pose.orientatoin.y = goal_q[1]
    ee_pose.orientatoin.z = goal_q[2]
    ee_pose.orientatoin.w = goal_q[3]

    ee_goal_msg.pose = ee_pose

    # call planToPose with ee_goal_msg
    ee_plan = moveit.planToPose(ee_goal_msg)

    # Execute plan to handover pose
    rob9Utils.iiwa.execute_spline_trajectory(ee_plan)
    rospy.sleep(2)
    #input("Press Enter when you are ready to move the robot back to the ready pose")

    gripper_pub.publish(open_gripper_msg)
    rospy.sleep(2)
    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))

def computeWaypoints(graspObjects, offset = 0.1):
    """ input:  graspsObjects   -   GraspGroup() of grasps
                offset          -   float, in meters for waypoint in relation to grasp
        output:                 -   nav_msgs Path
    """

    world_frame = "world"
    ee_frame = "right_ee_link"
    print("Computing waypoints for ", len(graspObjects), " task-oriented grasps.")

    grasps, waypoints = [], []
    for i in range(len(graspObjects)):

        grasp = graspObjects[i]

        graspCamera = copy.deepcopy(grasp)
        waypointCamera = copy.deepcopy(grasp)

        # computing waypoint in camera frame
        rotMat = graspCamera.getRotationMatrix()
        offsetArr = np.array([[offset], [0.0], [0.0]])
        offsetCam = np.transpose(np.matmul(rotMat, offsetArr))[0]

        waypointCamera.position.x += -offsetCam[0]
        waypointCamera.position.y += -offsetCam[1]
        waypointCamera.position.z += -offsetCam[2]

        waypointWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(waypointCamera.toPoseStampedMsg(), world_frame))
        graspWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(graspCamera.toPoseStampedMsg(), world_frame))

        waypointWorld.frame_id = str(graspObjects[i].tool_id) # we should probably do away with storing it in the header
        graspWorld.frame_id = str(graspObjects[i].tool_id)
        waypoints.append(waypointWorld.toPoseStampedMsg())
        grasps.append(graspWorld.toPoseStampedMsg())
        print(i+1, " / ", len(graspObjects))

    grasps_msg = nav_msgs.msg.Path()
    grasps_msg.header.frame_id = "world"
    grasps_msg.header.stamp = rospy.Time.now()
    for i in range(len(grasps)):
        grasps_msg.poses.append(waypoints[i])
        grasps_msg.poses.append(grasps[i])

    return grasps_msg

if __name__ == '__main__':
    global grasps_affordance, img, affClient, pcd, masks, bboxs, cloud_uv
    demo = std_msgs.msg.Bool()
    demo.data = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo.data = True
            print("Demo = True")
        else:
            print("Invalid input argument")
            exit()

    print("Init")
    rospy.init_node('moveit_subscriber', anonymous=True)

    set_ee = True
    if not rob9Utils.iiwa.setEndpointFrame():
        set_ee = False
    print("STATUS end point frame was changed: ", set_ee)

    set_PTP_speed_limit = True
    if not rob9Utils.iiwa.setPTPJointSpeedLimits():
        set_PTP_speed_limit = False
    print("STATUS PTP joint speed limits was changed: ", set_PTP_speed_limit)

    set_PTP_cart_speed_limit = True
    if not rob9Utils.iiwa.setPTPCartesianSpeedLimits():
        set_PTP_cart_speed_limit = False
    print("STATUS PTP cartesian speed limits was changed: ", set_PTP_cart_speed_limit)

    #rospy.Subscriber('tool_id', Int8, callback)
    rospy.Subscriber('objects_affordances_id', Int32MultiArray, callback )
    gripper_pub = rospy.Publisher('iiwa/gripper_controller', Int8, queue_size=10, latch=True)
    pub_grasp = rospy.Publisher('iiwa/pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('iiwa/pose_to_reach_waypoint', PoseStamped, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('iiwa/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # DO NOT REMOVE THIS SLEEP, it allows gripper_pub to establish connection to the topic
    rospy.sleep(0.1)
    rospy.sleep(2)

    vid_capture = cv2.VideoCapture(0)

    reset_gripper_msg = std_msgs.msg.Int8()
    reset_gripper_msg.data = 0
    activate_gripper_msg = std_msgs.msg.Int8()
    activate_gripper_msg.data = 1
    close_gripper_msg = std_msgs.msg.Int8()
    close_gripper_msg = 2
    open_gripper_msg = std_msgs.msg.Int8()
    open_gripper_msg.data = 3
    basic_gripper_msg = std_msgs.msg.Int8()
    basic_gripper_msg.data = 4
    pinch_gripper_msg = std_msgs.msg.Int8()
    pinch_gripper_msg.data = 5

    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    gripper_pub.publish(open_gripper_msg)
    gripper_pub.publish(pinch_gripper_msg)
    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))

    print("Services init")

    print("Camera is capturing new scene")

    cam = CameraClient()
    cam.captureNewScene()
    cloud, cloudColor = cam.getPointCloudStatic()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)

    cloud_uv = cam.getUvStatic()
    img = cam.getRGB()

    width = 1280
    height = 960
    h_divisions = 4
    w_divisions = 2
    viz_h_unit = int(height / h_divisions)
    viz_w_unit = viz_h_unit

    print("Generating grasps")

    # Get grasps from grasp generator
    graspClient = GraspingGeneratorClient()

    collision_thresh = 0.01 # Collision threshold in collision detection
    num_view = 300 # View number
    score_thresh = 0.0 # Remove every grasp with scores less than threshold
    voxel_size = 0.2

    graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

    # Load the network with GPU (True or False) or CPU
    graspClient.start(GPU=True)

    graspData = graspClient.getGrasps()

    print("Got ", len(graspData), " grasps")

    all_grasp_viz_internal = visualizeGrasps6DOF(pcd, graspData)
    o3d.visualization.draw_geometries([pcd, *all_grasp_viz_internal])

    print("Segmenting affordance maps")
    # Run affordance analyzer
    affClient = AffordanceClient()

    affClient.start(GPU=False)
    _ = affClient.run(img, CONF_THRESHOLD = 0.5)

    masks, labels, scores, bboxs = affClient.getAffordanceResult()

    masks = affClient.processMasks(masks, conf_threshold = 0, erode_kernel=(1,1))

    # Visualize object detection and affordance segmentation to confirm
    cv2.imwrite("rgb.png", img)
    cv2.imshow("Detections", affClient.visualizeBBox(img, labels, bboxs, scores))
    cv2.imwrite("detections.png", affClient.visualizeBBox(img, labels, bboxs, scores))
    cv2.waitKey(0)
    cv2.imshow("mask", visualizeMasksInRGB(img, masks))
    cv2.waitKey(0)

    print("Computing task oriented grasps")

    # Associate affordances with grasps
    grasps_affordance = associateGraspAffordance(graspData, labels, masks, cloud, cloud_uv, demo = demo.data)

    print("Found ", len(grasps_affordance), " task oriented grasps")

    task_grasp_viz_internal = visualizeGrasps6DOF(pcd, grasps_affordance)
    o3d.visualization.draw_geometries([pcd, *task_grasp_viz_internal])

    grasps_affordance.sortByScore()
    grasps_affordance.thresholdByScore(0.0)

    print("Ready for command")


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
