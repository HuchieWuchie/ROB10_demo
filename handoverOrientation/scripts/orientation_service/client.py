#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from orientation_service.srv import runOrientation, runOrientationResponse

class OrientationClient(object):
    """docstring for orientationClient."""

    def __init__(self):

        self.method = 0 # learned from observation

    def getOrientation(self, pcd, uv, masks, bbox = None):

        rospy.wait_for_service("/computation/handover_orientation/get")
        orientationService = rospy.ServiceProxy("/computation/handover_orientation/get", runOrientationSrv)

        msg = runOrientationSrv()
        camClient = CameraClient()
        affClient = AffordanceClient()

        msg.pcd = camClient.packPCD(pcd)
        msg.uv = camClient.packUV(uv)
        msg.masks = affClient.packMasks(masks)

        if bbox is not None:
            msg.bbox = affClient.packBbox(bbox)

        response = orientationService(msg)

        orientation = self.unpackOrientation(response.orientation)

        del response

        return grasps

    def setSettings(self):

        todo = True

    def packOrientation(self, orientation):

        msg = Float32MultiArray()
        orientation = orientation.flatten().tolist()
        msg.data = orientation

        return msg

    def unpackOrientation(msg):
        return np.asarray(msg.data).reshape((3,3))
