#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import open3d as o3d
import os
import time

from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from orientationService.client import OrientationClient

from rob9Utils.visualize import visualizeMasksInRGB, visualizeFrameMesh


if __name__ == "__main__":
    print("Usage example of client server service")
    rospy.init_node('handover_orientation_client', anonymous=True)

    rotClient = OrientationClient()

    # Load sample data

    path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "sample_data")

    masks = np.load(os.path.join(path, "masks.npy")).astype(np.uint8)
    labels = np.load(os.path.join(path, "labels.npy"))
    bboxs = np.load(os.path.join(path, "bboxs.npy"))
    uv = np.load(os.path.join(path, "uv.npy"))
    pcd_colors = np.load(os.path.join(path, "cloudColor.npy"))
    pcd = o3d.io.read_point_cloud(os.path.join(path, "pcd.ply"))
    geometry = np.asanyarray(pcd.points)
    img = cv2.imread(os.path.join(path, "img.png"))

    # Select object 5, ladle or 0 for knife

    masks = masks[5]
    labels = labels[5]
    bboxs = bboxs[5]

    # Visualize masks
    cv2.imshow("Masks", visualizeMasksInRGB(img, masks))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Compute orientation
    print("computing")

    ts = time.time()
    orientation, translation, goal = rotClient.getOrientation(geometry, uv, masks, bboxs)
    te = time.time()

    t_2_f = te - ts
    print("Computed current transformation and goal orientation in: ", round(t_2_f, 2), " seconds")

    print(orientation)
    print(translation)
    print(goal)

    # Visualize computed orientation
    rotated_coordinate_frame = visualizeFrameMesh(translation, orientation)
    #rotated_coordinate_frame_to_goal = visualizeFrameMesh(translation, goal)

    o3d.visualization.draw_geometries([pcd, rotated_coordinate_frame])
