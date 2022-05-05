#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import numpy as np
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from orientation_service.srv import runOrientationSrv, runOrientationSrvResponse

class OrientationClient(object):
    """docstring for orientationClient."""

    def __init__(self):

        self.method = 0 # learned from observation

    def getOrientation(self, pcd, uv, masks, bbox = None):

        print("Waiting for orientation service...")
        rospy.wait_for_service("/computation/handover_orientation/get")
        print("Orientation service is up...")
        orientationService = rospy.ServiceProxy("/computation/handover_orientation/get", runOrientationSrv)
        print("Connection to orientation service established!")

        camClient = CameraClient()
        affClient = AffordanceClient(connected = False)

        pcd_msg, _ = camClient.packPCD(pcd, 0)
        uv_msg = camClient.packUV(uv)
        masks_msg = affClient.packMasks(masks)

        bbox_msg = Int32MultiArray()
        if bbox is not None:
            bbox_msg = affClient.packBbox(bbox)

        print("Message constructed")

        response = orientationService(pcd_msg, uv_msg, masks_msg, bbox_msg)

        current_orientation, current_translation, goal_orientation = self.unpackOrientation(response.current, response.goal)

        del response

        return current_orientation, current_translation, goal_orientation

    def setSettings(self):

        todo = True

    def packOrientation(self, current_transformation, goal_orientation):

        msg_current = Float32MultiArray()
        current_transformation = current_transformation.flatten().tolist()
        msg_current.data = current_transformation

        msg_goal = Float32MultiArray()
        goal_orientation = goal_orientation.flatten().tolist()
        msg_goal.data = goal_orientation

        return msg_current, msg_goal

    def unpackOrientation(self, msg_current, msg_goal):
        current_transformation = np.asarray(msg_current.data).reshape((4,4))
        orientation = current_transformation[:3,:3]
        translation = current_transformation[:3,3]

        goal = np.asarray(msg_goal.data).reshape((3,3))
        return orientation, translation, goal
