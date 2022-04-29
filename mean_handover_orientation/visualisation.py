# https://rock-learning.github.io/pytransform3d/index.html
# https://rock-learning.github.io/pytransform3d/_modules/pytransform3d/transform_manager.html#TransformManager
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

import matplotlib.pyplot as plt
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transformations import random_transform
from pytransform3d.transform_manager import TransformManager
from pytransform3d.transformations import plot_transform

def fix_transformation(transformation):
    #print(transformation)
    #print("===========")
    #rot_mat = np.zeros((3,3))
    #rot_mat = transformation[:3, :3]
    #print(transformation)
    fixed_transformation = np.zeros((4,4))
    fixed_transformation[3,3] = 1

    #for i in range(0, 3):
        #for j in range(0, 3):
            #rot_mat[i,j] = transformation[i,j]
    #print(rot_mat)

    #rot_vector = R.from_matrix(transformation[:3, :3]).as_rotvec()
    q = R.from_matrix(transformation[:3, :3]).as_quat()
    #print(rot_vector)
    # MAY NOT BE NECESSARY, SEEMS TO BE NORMALIZED
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    #print(q)
    #print("===========")
    fixed_rot_mat = R.from_quat(q).as_matrix()
    #q_fixed = R.from_matrix(fixed_rot_mat).as_quat()
    #print(rot_vector)
    #print(q_fixed)
    #print("===========")

    fixed_transformation[:3, :3] = fixed_rot_mat
    #print(fixed_transformation)
    #print("===========")
    #temp = np.matmul(fixed_rot_mat, fixed_rot_mat.transpose())
    #det = np.linalg.det(fixed_rot_mat)
    #print(temp)
    #print(det)
    #exit(2)
    return fixed_transformation

def get_quat_from_matrix(transformation):
    #print(transformation)
    rot_mat = np.zeros((3,3))
    rot_mat = transformation[:3, :3]
    #for i in range(0, 3):
        #for j in range(0, 3):
            #rot_mat[i,j] = transformation[i,j]
    #print(rot_mat)
    q = R.from_matrix(rot_mat).as_quat()
    # MAY NOT BE NECESSARY, SEEMS TO BE NORMALIZED
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    return q

if __name__ == '__main__':
    tm = TransformManager()
    path = "/home/daniel/Transformations/transformations_parsed"
    output_path = "/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_handover_orientations.txt"
    root, dirs, _ = next(os.walk(path))
    for dir in dirs:
        _, _, files = next(os.walk(os.path.join(root,dir)))
        number_of_files = len(files)
        #observations = np.zeros((number_of_files, 4))
        #print(np.shape(observations))
        temp = []
        for i in range(0, number_of_files):
            rot_mat = np.zeros((3,3))
            transformation = np.load(os.path.join(root,dir,files[i]))
            fixed_transformation = fix_transformation(transformation)
            temp.append(fixed_transformation)
            #print(transformation)
            #print("=============")
            #name = "observation_" + str(i)
            #print(type(ax))
            #plt.tight_layout()

        ax = make_3d_axis(7)
        for transformation in fixed_transformation:
            ax.plot(transformation)
            #
            #tm.add_transform("world", None, fixed_transformation)
            #observations[i, :] = get_quat_from_matrix(transformation)
        #plt.show()
        #plt.figure(figsize=(10, 5))
        #plt.figure()
        #ax = make_3d_axis()
        #ax = tm.plot_frames_in("world", ax=ax, alpha=0.6)
        #ax.view_init(30, 20)
        #plt.show()
        plt.show()
        exit(3)
        #solution = find_solution(observations)

"""
random_state = np.random.RandomState(0)
A2world = random_transform(random_state)
B2world = random_transform(random_state)
A2C = random_transform(random_state)
D2B = random_transform(random_state)

tm = TransformManager()
tm.add_transform("A", "world", A2world)
tm.add_transform("B", "world", B2world)
tm.add_transform("A", "C", A2C)
tm.add_transform("D", "B", D2B)

plt.figure(figsize=(10, 5))

ax = make_3d_axis(3, 121)
ax = tm.plot_frames_in("world", ax=ax, alpha=0.6)
ax.view_init(30, 20)

ax = make_3d_axis(3, 122)
ax = tm.plot_frames_in("A", ax=ax, alpha=0.6)
ax.view_init(30, 20)

plt.show()
"""
