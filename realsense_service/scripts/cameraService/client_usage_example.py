#!/usr/bin/env python3

"""
Import sys modular, sys.argv The function of is to pass parameters from the outside to the inside of the program. sys.argv(number)，number=0 Is the name of the script
"""
import sys
import rospy
from realsense_service.srv import *
import numpy as np
import cv2
import open3d as o3d
from cv_bridge import CvBridge
from cameraClient import CameraClient

if __name__ == "__main__":

    print("Starting")
    cam = CameraClient(type = "realsenseD435")
    counter = 0

    while(counter<100):
        print("Capture new scene " + str(counter))
        counter += 1;

        cam.captureNewScene()

        cam.getRGB()
        #cv2.imshow("rgb image", cam.rgb)
        #cv2.waitKey(0)

        # SOMETHING WRONG in cam.getDepth() -> ... dtype=np.float16 ...
        cam.getDepth()
        #print(cam.depth)
        #print(np.unique(cam.depth))
        #cv2.imshow("depth image", (cam.depth*255).astype(np.uint8))
        #cv2.waitKey(0)

        cam.getUvStatic()
        #print(cam.uv.shape)
        #print(cam.uv[0:10000])

        cloud, rgb = cam.getPointCloudStatic()
        #pcd = o3d.geometry.PointCloud()
        #pcd.points = o3d.utility.Vector3dVector(cloud)
        #pcd.colors = o3d.utility.Vector3dVector(rgb)
        #o3d.visualization.draw_geometries([pcd])

        #rospy.sleep(0.1)
