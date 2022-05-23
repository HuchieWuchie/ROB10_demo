#!/usr/bin/env python3
import rospy
from affordance_analyzer.srv import getAffordanceSrv, getAffordanceSrvResponse
import numpy as np
import cv2
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from rob9Utils.visualize import visualizeMasksInRGB

import open3d as o3d
import copy

if __name__ == "__main__":
    print("Usage example of client server service")
    rospy.init_node('affordance_analyzer_client', anonymous=True)

    affClient = AffordanceClient()
    cam = CameraClient()
    cam.captureNewScene()
    cloud, cloudColor = cam.getPointCloudStatic()
    cloud_uv = cam.getUvStatic()
    img = cam.getRGB()

    cv2.imshow("name", img)
    cv2.waitKey(0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)
    affClient.start(GPU=True) # GPU acceleratio True or False
    print(affClient.name)
    print("Telling affordanceNET to analyze image from realsense and return predictions.")
    success = affClient.run(img, CONF_THRESHOLD = 0.7)
    masks, labels, scores, bboxs = affClient.getAffordanceResult()
    print("Got result")

    print(scores)

    masks = affClient.processMasks(masks, conf_threshold = 0, erode_kernel=(1,1))
    #masks = affClient.processMasks(conf_threshold = 230, erode_kernel=(3,3))
    #affClient.masks = masks
    #width, height = img.shape[1], img.shape[0]
    #ratio = width / height
    #img = cv2.resize(img, (int(450 * ratio), 450), interpolation = cv2.INTER_AREA)
    #img = affClient.visualizeMasks(img, masks)
    #img = affClient.visualizeBBox(img, labels, bboxs, scores)
    cv2.imshow("output", affClient.visualizeBBox(visualizeMasksInRGB(img, masks), labels, bboxs, scores))
    cv2.waitKey(0)

    if True:
        np.save("masks.npy", masks.astype(np.bool))
        np.save("labels.npy", labels)
        np.save("bboxs.npy", bboxs)
        np.save("scores.npy", scores)
        np.save("cloudColor.npy", cloudColor)
        np.save("uv.npy", cloud_uv)
        cv2.imwrite("img.png", img)
        o3d.io.write_point_cloud("pcd.ply", pcd)


    clouds_m = []

    cv2.imwrite("img.png", img)

    print("Found the following objects")
    for i in range(len(affClient.objects)):
        print(affClient.OBJ_CLASSES[affClient.objects[i]])
