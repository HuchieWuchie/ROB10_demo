#!/usr/bin/env python3

import os
import rospy
import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.transform import Rotation as R
import random

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header, Float32

from orientation_service.srv import runOrientationSrv, runOrientationSrvResponse

from rob9Utils.affordancetools import getPredictedAffordances, getAffordanceColors, getAffordanceContours, getObjectAffordancePointCloud
from rob9Utils.utils import erodeMask, keepLargestContour, convexHullFromContours, maskFromConvexHull, thresholdMaskBySize, removeOverlapMask

from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from orientationService.client import OrientationClient

class OrientationServer(object):
    """docstring for OrientationServer."""

    def __init__(self):

        print('Starting...')
        rospy.init_node('orientation_service', anonymous=True)
        self.serviceRun = rospy.Service("/computation/handover_orientation/get", runOrientationSrv, self.run)

        self.rate = rospy.Rate(5)

        root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "sampled_object_point_clouds")
        self.pcd_paths = []
        self.pcd_paths.append(os.path.join(root_dir, "14_spatula_14.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "1_knife_8.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "15_hammer_13.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "5_scoop_5.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "10_ladle_7.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "9_cup_16.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "11_mug_1.txt"))
        self.pcd_paths.append(os.path.join(root_dir, "18_bottle_3.txt"))

        self.mean_quat = []
        self.mean_quat.append([ 0.17858507, -0.72325377,  0.14493605,  0.65115659]) # spatula
        self.mean_quat.append([ 0.05262527, -0.72957124,  0.15906051,  0.66306572]) # knife
        self.mean_quat.append([ 0.04510172,  0.6002882,  -0.32359202, -0.73000556]) # hammer
        self.mean_quat.append([ 0.09140731, -0.71904327,  0.15291612,  0.67174261]) # scoop
        self.mean_quat.append([-0.03664714,  0.72422009, -0.08418687, -0.68342872]) # ladle
        self.mean_quat.append([-0.0985996,   0.70367063 ,-0.18686055, -0.67838698]) # cup
        self.mean_quat.append([ 0.25167641,  0.69930462,  0.32368484, -0.58554261]) # mug
        self.mean_quat.append([-0.66268984, -0.04537715, -0.68318855,  0.30337519]) # bottle

        self.nn_classifier = self.createNNClassifier()

        self.method = 0 # observation based

    def setSettings(self, msg):

        todo = True
        return setSettingsOrientationSrvResponse()

    def methodObservation(self, points, uv, masks, bbox = None):

        affordances_in_object = getPredictedAffordances(masks = masks, bbox = bbox)
        print("predicted affordances", affordances_in_object)

        for aff in affordances_in_object:

            masks = erodeMask(affordance_id = aff, masks = masks,
                            kernel = np.ones((3,3)))
            contours = getAffordanceContours(bbox = bbox, affordance_id = aff,
                                            masks = masks)
            if len(contours) > 0:
                contours = keepLargestContour(contours)
                hulls = convexHullFromContours(contours)

                h, w = masks.shape[-2], masks.shape[-1]
                if bbox is not None:
                    h = int(bbox[3] - bbox[1])
                    w = int(bbox[2] - bbox[0])

                aff_mask = maskFromConvexHull(h, w, hulls = hulls)
                _, keep = thresholdMaskBySize(aff_mask, threshold = 0.05)
                if keep == False:
                    aff_mask[:, :] = False

                if bbox is not None:
                    masks[aff, bbox[1]:bbox[3], bbox[0]:bbox[2]] = aff_mask
                else:
                    masks[aff, :, :] = aff_mask

        masks = removeOverlapMask(masks = masks)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd_affordance = getObjectAffordancePointCloud(pcd, masks, uvs = uv)

        feature_vector = self.computeFeatureVector(pcd_affordance)
        dist, predictions = self.nn_classifier.kneighbors(feature_vector, n_neighbors = 1)
        source_pcd = self.getSourcePointCloud(prediction = predictions[0][0])

        target_bounds = pcd_affordance.get_max_bound() - pcd_affordance.get_min_bound()
        source_bounds = source_pcd.get_max_bound() - source_pcd.get_min_bound()
        scale = np.max(target_bounds) / np.max(source_bounds)

        source_pcd_points = np.asarray(source_pcd.points)
        source_pcd_points = source_pcd_points * scale
        source_pcd.points = o3d.utility.Vector3dVector(source_pcd_points)

        T, distances, iterations = self.icp(source_points = np.asanyarray(source_pcd.points),
                                        source_colors = np.asanyarray(source_pcd.colors),
                                        target_points = np.asanyarray(pcd_affordance.points),
                                        target_colors = np.asanyarray(pcd_affordance.colors),
                                        tolerance=0.00001)

        current_position = pcd_affordance.get_center()
        T[:3,3] = current_position

        source_pcd.transform(T)
        return T, self.getGoalOrientation(predictions[0][0])

    def run(self, msg):

        print("received request...")

        camClient = CameraClient()
        pcd, _ = camClient.unpackPCD(msg_geometry = msg.pcd, msg_color = None)
        uv = camClient.unpackUV(msg.uv)

        affClient = AffordanceClient(connected = False)
        masks = affClient.unpackMasks(msg.masks)
        bbox = affClient.unpackBBox(msg.bbox)

        T = 0
        G = 0
        if self.method == 0:
            T, G = self.methodObservation(pcd, uv, masks[0], bbox[0])

        np.set_printoptions(suppress=True)
        print(T)
        print()
        print(G)

        msg = runOrientationSrvResponse()
        rotClient = OrientationClient()
        msg.current, msg.goal = rotClient.packOrientation(T, G)

        return msg

    def createNNClassifier(self):

        # bg, grasp, cut, scoop, contain, pound, support, w-grasp
        features = []
        features.append([0, 1, 0, 0, 0, 0, 1, 0]) # spatula, shovel
        features.append([0, 1, 1, 0, 0, 0, 0, 0]) # saw, knife, scissors, shears
        features.append([0, 1, 0, 0, 0, 1, 0, 0]) # hammer, mallet, tenderizers
        features.append([0, 1, 0, 1, 0, 0, 0, 0]) # scoop, spoon, trowel
        features.append([0, 1, 0, 0, 1, 0, 0, 0]) # ladle
        features.append([0, 0, 0, 0, 1, 0, 0, 1]) # bowl, cup
        features.append([0, 1, 0, 0, 1, 0, 0, 1]) # mug
        features.append([0, 1, 0, 0, 0, 0, 0, 0]) # bottle

        features = np.array(features)

        nn_classifier = NearestNeighbors(n_neighbors =  1).fit(features)

        return nn_classifier

    def getSourcePointCloud(self, prediction):

        path = self.pcd_paths[prediction]
        pcd = o3d.io.read_point_cloud(path, format='xyzrgb')

        return pcd

    def getGoalOrientation(self, prediction):

        rotation = R.from_quat(self.mean_quat[prediction])
        rotation_matrix = rotation.as_matrix()

        return rotation_matrix

    def computeFeatureVector(self, pcd):

        label_colors = {(0, 0, 255): 1, # grasp
        (0, 255, 0): 2, # cut
        (123, 255, 123): 3, # scoop
        (255, 0, 0): 4, # contain
        (255, 255, 0): 5, # pound
        (255, 255, 255): 6, # support
        (255, 0, 255): 7} # wrap-grasp

        feature_vector = np.zeros((1,8))

        pcd_colors = np.asanyarray(pcd.colors).astype(np.uint8)
        if np.max(pcd_colors) <= 1:
            pcd_colors = pcd_colors * 255

        for label_color in label_colors:
            idx = pcd_colors == label_color
            idx = np.sum(idx, axis = -1) == 3
            if True in idx:
                feature_vector[0, label_colors[label_color]] = 1

        return feature_vector


    def best_fit_transform(self, A, B):
        '''
        https://github.com/ClayFlannigan/icp
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
          A: Nxm numpy array of corresponding points
          B: Nxm numpy array of corresponding points
        Returns:
          T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
          R: mxm rotation matrix
          t: mx1 translation vector
        '''

        assert A.shape == B.shape

        # get number of dimensions
        m = A.shape[1]

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
           Vt[m-1,:] *= -1
           R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R,centroid_A.T)

        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t

        return T, R, t


    def nearest_neighbor(self, src, dst):
        '''
        https://github.com/ClayFlannigan/icp
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: dst indices of the nearest neighbor
        '''

        assert src.shape == dst.shape

        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()


    def icp(self, source_points, source_colors, target_points, target_colors, init_pose=None, max_iterations=100, tolerance=0.0001):
        '''
        https://github.com/ClayFlannigan/icp
        The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
        Input:
            A: Nxm numpy array of source mD points
            B: Nxm numpy array of destination mD point
            init_pose: (m+1)x(m+1) homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation that maps A on to B
            distances: Euclidean distances (errors) of the nearest neighbor
            i: number of iterations to converge
        '''

        if source_points.shape[0] > target_points.shape[0]:
            indices = t_indices = random.sample(range(0, source_points.shape[0]), target_points.shape[0])
            source_points = source_points[indices]
            source_colors = source_colors[indices]

        elif target_points.shape[0] > source_points.shape[0]:
            indices = t_indices = random.sample(range(0, target_points.shape[0]), source_points.shape[0])
            target_points = target_points[indices]
            target_colors = target_colors[indices]

        assert source_points.shape == target_points.shape
        A = source_points
        B = target_points

        # get number of dimensions
        m = A.shape[1]

        # make points homogeneous, copy them to maintain the originals
        src = np.ones((m+1,A.shape[0]))
        dst = np.ones((m+1,B.shape[0]))
        src[:m,:] = np.copy(A.T)
        dst[:m,:] = np.copy(B.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            src_aff = np.hstack((src[:m,:].T, source_colors))
            distances, indices = self.nearest_neighbor(np.hstack((src[:m,:].T, source_colors)),
                                                    np.hstack((dst[:m,:].T, target_colors)))

            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[:m,:].T, dst[:m,indices].T)

            # update the current source
            src = np.dot(T, src)

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculate final transformation
        T,_,_ = self.best_fit_transform(A, src[:m,:].T)

        return T, distances, i

if __name__ == "__main__":

    orientation_predictor = OrientationServer()
    print("Handover orientation service is ready")

    while not rospy.is_shutdown():
        rospy.spin()
