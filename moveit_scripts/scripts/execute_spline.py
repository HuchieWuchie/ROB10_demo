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

def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = moveit.getCurrentState()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def callback(msg):
    global req_obj_id, req_aff_id

	req_obj_id = msg.data[0]
    req_aff_id = msg.data[1]

def computeWaypoint(grasp, offset = 0.1):
    """ input:  graspsObjects   -   rob9Utils.grasp.Grasp() in world_frame
                offset          -   float, in meters for waypoint in relation to grasp
        output:
				waypoint		-   rob9Utils.grasp.Grasp()
    """

    world_frame = "world"
    ee_frame = "right_ee_link"

    waypoint = copy.deepcopy(grasp)

	# you can implement some error handling here if the grasp is given in the wrong frame
	#waypointWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(waypointCamera.toPoseStampedMsg(), world_frame))
	#graspWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(graspCamera.toPoseStampedMsg(), world_frame))

    # computing waypoint in camera frame
    rotMat = grasp.getRotationMatrix()
    offsetArr = np.array([[0.0], [0.0], [offset]])
    offsetCam = np.transpose(np.matmul(rotMat, offsetArr))[0]

    waypoint.position.x += -offsetCam[0]
    waypoint.position.y += -offsetCam[1]
    waypoint.position.z += -offsetCam[2]

    return waypoint

def wait():
	try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    global grasps_affordance, img, affClient, pcd, masks, bboxs, req_aff_id, req_obj_id, state

    print("Init")
    rospy.init_node('moveit_subscriber', anonymous=True)

	state = 1 # start at setup phase

	while True:

		if state == 1:
			# setup phase

		    set_ee = True
		    if not rob9Utils.iiwa.setEndpointFrame():
		        set_ee = False
		    print("STATUS end point frame was changed: ", set_ee)

		    set_PTP_speed_limit = True
		    if not rob9Utils.iiwa.setPTPJointSpeedLimits():
		        set_PTP_speed_limit = False
		    print("STATUS PTP joint speed limits was changed: ", set_PTP_speed_limit)

<<<<<<< HEAD
def callback(msg):
    global resp_trajectories, grasps_affordance, pcd, masks, cloud_uv, bboxs

    id = msg.data[0]
    requested_affordance_id = msg.data[1]
=======
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
>>>>>>> c175c54cce599769f2679d104239817feddf84cb

		    vid_capture = cv2.VideoCapture(0)

<<<<<<< HEAD
    # Select grasps of objects with correct affordance
    graspObj = GraspGroup(grasps = copy.deepcopy(grasps_affordance.getgraspsByTool(id = id)))
    grasps_affordance_tool = GraspGroup(grasps = copy.deepcopy(graspObj.getgraspsByAffordanceLabel(label = requested_affordance_id)))
=======
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
>>>>>>> c175c54cce599769f2679d104239817feddf84cb

		    gripper_pub.publish(reset_gripper_msg)
		    gripper_pub.publish(activate_gripper_msg)
		    gripper_pub.publish(open_gripper_msg)
		    gripper_pub.publish(pinch_gripper_msg)
		    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))

			req_obj_id = -1
			req_aff_id = -1

		    print("Services init")

			state = 2

		elif state == 2:
			# Capture sensor information

		    print("Camera is capturing new scene")

		    cam = CameraClient()
		    cam.captureNewScene()
		    cloud, cloudColor = cam.getPointCloudStatic()
		    pcd = o3d.geometry.PointCloud()
		    pcd.points = o3d.utility.Vector3dVector(cloud)
		    pcd.colors = o3d.utility.Vector3dVector(cloudColor)

		    cloud_uv = cam.getUvStatic()
		    img = cam.getRGB()

			state = 3

		elif state == 3:
			# Analyze affordance

<<<<<<< HEAD
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
=======
			print("Segmenting affordance maps")
		    affClient = AffordanceClient()

		    affClient.start(GPU=True)
		    _ = affClient.run(img, CONF_THRESHOLD = 0.5)

		    masks, labels, scores, bboxs = affClient.getAffordanceResult()
		    masks = affClient.processMasks(masks, conf_threshold = 0, erode_kernel=(1,1))

			state = 4

		elif state == 4:
			while req_aff_id == -1 and req_obj_id == -1:
				wait()
			state = 5

		elif state == 5:
			# Check user input
			num_occurences = np.where(labels == req_obj_id)[0].shape[0]
			if num_occurences == 0:
				req_obj_id = -1
				req_obj_id = -1

			state = 6

		elif state == 6:
			# post process affordance segmentation maps

			obj_inst = np.where(labels == req_obj_id)[0][0]
		    obj_inst_masks = masks[obj_inst]
		    obj_inst_labels = labels[obj_inst]
		    obj_inst_bbox = bboxs[obj_inst]

			# Post process affordance predictions and compute point cloud affordance mask

		    affordances_in_object = getPredictedAffordances(masks = obj_inst_masks, bbox = obj_inst_bbox)
		    print("predicted affordances", affordances_in_object)
>>>>>>> c175c54cce599769f2679d104239817feddf84cb

		    for aff in affordances_in_object:

		        masks = erodeMask(affordance_id = req_aff_id, masks = obj_inst_masks,
		                        kernel = np.ones((3,3)))
		        contours = getAffordanceContours(bbox = obj_inst_bbox, affordance_id = req_aff_id,
		                                        masks = obj_inst_masks)
		        if len(contours) > 0:
		            contours = keepLargestContour(contours)
		            hulls = convexHullFromContours(contours)

		            h, w = obj_inst_masks.shape[-2], obj_inst_masks.shape[-1]
		            if obj_inst_bbox is not None:
		                h = int(obj_inst_bbox[3] - obj_inst_bbox[1])
		                w = int(obj_inst_bbox[2] - obj_inst_bbox[0])

<<<<<<< HEAD
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
=======
		            aff_mask = maskFromConvexHull(h, w, hulls = hulls)
		            _, keep = thresholdMaskBySize(aff_mask, threshold = 0.05)
		            if keep == False:
		                aff_mask[:, :] = False

		            if bbox is not None:
		                obj_inst_masks[aff, bbox[1]:bbox[3], bbox[0]:bbox[2]] = aff_mask
		            else:
		                obj_inst_masks[aff, :, :] = aff_mask

		    obj_inst_masks = removeOverlapMask(masks = obj_inst_masks)
			pcd_affordance = getObjectAffordancePointCloud(pcd, obj_inst_masks, uvs = uv)
			state = 7

		elif state == 7
			# transform point cloud into world coordinate frame
		    # below is a transformation used during capture of sample data

		    cam2WorldTransform = np.array([-(math.pi/2) - 1.0995574287564276, 0, -(math.pi)-(math.pi/4)+1.2220795422464295])
		    rotCam2World = R.from_euler('xyz', cam2WorldTransform)
		    rotMatCam2World = rotCam2World.as_matrix()

		    points = np.dot(rotMatCam2World, np.asanyarray(pcd.points).T).T
		    pcd.points = o3d.utility.Vector3dVector(points)

		    # Compute a downsampled version of the point cloud for collision checking
		    # downsampling speeds up computation
		    pcd_downsample = pcd.voxel_down_sample(voxel_size=0.005)
>>>>>>> c175c54cce599769f2679d104239817feddf84cb

			state = 8

		elif state == 8

		    # Select affordance mask to compute grasps for
		    observed_affordances = getPredictedAffordances(obj_inst_masks)

		    success, sampled_grasp_points = getPointCloudAffordanceMask(affordance_id = req_aff_id,
		                                    points = points, uvs = uv, masks = obj_inst_masks)

			if success:

				# computing goal pose of object in world frame and
				# current pose of object in world frame

				rotClient = OrientationClient()
				current_orientation, current_position, goal_orientation = rotClient.getOrientation(pcd_affordance) # we discard translation

				curr_rot_quat = transform.quaternionFromRotation(current_orientation)
				curr_pose_world = np.hstack((current_position.flatten(), curr_rot_quat))

		        # run the algorithm
				grasp_client = GraspingGeneratorClient()
		        grasp_group = grasp_client.run(sampled_grasp_points, pcd_downsample,
		                                    "world", req_obj_id, req_aff_id, obj_inst)

		        print("I got ", len(grasp_group.grasps), " grasps")

		        grasp_group.thresholdByScore(0.3)
		        grasp_group.sortByScore()

				for grasp in grasp_group:

<<<<<<< HEAD
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
=======
					waypoint = computeWaypoint(grasp, offset = 0.1)
					waypoint_pose_msg = waypoint.toPoseMsg()
>>>>>>> c175c54cce599769f2679d104239817feddf84cb

					plan_found_waypoint, plan_waypoint = moveit.planToPose(waypoint_msg)
					if plan_found_waypoint:
						grasp_msg = grasp.toPoseMsg()
						plan_found_waypoint_to_grasp, plan_grasp = moveit.planFromPoseToPose(waypoint_msg, rasp_msg)

						if plan_found_waypoint_to_grasp:

							pub_waypoint.publish(waypoint.toPoseStampedMsg())
							pub_grasp.publish(grasp.toPoseStampedMsg())

							## Get goal position using linescan service

							goal_rot_quat = transform.quaternionFromRotation(goal_orientation)

							locClient = LocationClient()
							goal_location = locClient.getLocation()
							goal_location_giver = transform.transformToFrame(curr_pose, "giver", "world")
							goal_pose_giver = np.hstack((goal_location.flatten(), goal_rot_quat))

							goal_pose_world = transform.transformToFrame(goal_pose_giver, "world", "giver")

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

							ee_pose.orientation.x = goal_q[0]
							ee_pose.orientation.y = goal_q[1]
							ee_pose.orientation.z = goal_q[2]
							ee_pose.orientation.w = goal_q[3]

							ee_goal_msg.pose = ee_pose

							# call planToPose with ee_goal_msg
							plan_found_handover, plan_handover = moveit.planToPose(ee_goal_msg)

							if plan_found_handover:

						        rob9Utils.iiwa.execute_spline_trajectory(plan_waypoint)
								rospy.sleep(1)

								rob9Utils.iiwa.execute_spline_trajectory(plan_grasp)
								rospy.sleep(1)

								gripper_pub.publish(close_gripper_msg)
								rospy.sleep(1)

								print("I have grasped!")
								#input("Press Enter when you are ready to move the robot back to the ready pose") # outcommented by Albert Wed 23 March 09:06

							    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))

								# Execute plan to handover pose
								rob9Utils.iiwa.execute_spline_trajectory(plan_handover)
							    rospy.sleep(2)
							    #input("Press Enter when you are ready to move the robot back to the ready pose")

							    gripper_pub.publish(open_gripper_msg)
							    rospy.sleep(2)
							    rob9Utils.iiwa.execute_spline_trajectory(moveit.planToNamed("ready"))
								break

			state = 9

		elif state == 9:
			# restart
			req_aff_id = -1
			req_obj_id = -1

			state = 2

		try:
	        rospy.spin()
	    except rospy.ROSInterruptException:
	        pass
	#rotMat = grasp.orientation.getRotationMatrix()
    #translation = grasp.position.getVector()


    #gripper = createGripper(opening = 0.08, translation = translation, rotation = rotMat)
    #vis_gripper = visualizeGripper(gripper)





    #print("I got ", len(grasp_group.grasps), " grasps after thresholding")

    #vis_grasps = visualizeGrasps6DOF(pcd, grasp_group)
    #o3d.visualization.draw_geometries([pcd, *vis_grasps, vis_gripper])
