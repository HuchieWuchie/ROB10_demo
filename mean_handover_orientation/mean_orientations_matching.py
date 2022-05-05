import os
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_mean_orientation(class_name):
    with open('/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_handover_orientations.txt') as f:
        lines = f.readlines()

    mean_orientation = np.zeros(4)
    for line in lines:
        line = line.replace("[","")
        line = line.replace("]","")
        temp = line.split()
        if class_name == temp[0]:
            #print(temp)
            mean_orientation = temp[1:5]
            for i in range(0,4):
                mean_orientation[i] = float(mean_orientation[i])
    mean_orientation = np.asarray(mean_orientation)
    return mean_orientation

def rotate_frame(rotation):
    #print("===ROTATION===\n",rotation)
    rot_mat = R.from_quat(rotation).as_matrix()
    #print("===ROTATION MAT===\n", rot_mat)
    unit_frame = np.eye(3)
    #print("===UNIT FRAME===\n",unit_frame)
    rotated_frame = np.matmul(unit_frame, rot_mat)
    #print("===ROTATED FRAME===\n", rotated_frame)
    return rotated_frame

def get_distances(group, frames, axis):
    distances = []
    for class_id in group:
        index = group.index(class_id)
        if index == len(group):
            break
        for i in range(index+1, len(group)):
            delta = frames[index] - frames[i]
            distance = np.linalg.norm(delta[:,axis])
            distances.append(distance)
    return distances

if __name__ == '__main__':
    groupAB = ["spatula", "knife", "hammer", "scissors", "scoop", "mallet", "ladle", "spoon"]
    groupC = ["cup", "mug", "bowl", "bottle"]

    rotated_frames_AB = []
    rotated_frames_C = []

    for class_id in groupC:
        mean_orientation = get_mean_orientation(class_id)
        rotated_frame = rotate_frame(mean_orientation)
        rotated_frames_C.append(rotated_frame)
    rotated_frames_C = np.asarray(rotated_frames_C)
    print("===ROTATED FRAMES GROUP C SHAPE===\n", np.shape(rotated_frames_C))

    for class_id in groupAB:
        mean_orientation = get_mean_orientation(class_id)
        rotated_frame = rotate_frame(mean_orientation)
        rotated_frames_AB.append(rotated_frame)
    rotated_frames_AB = np.asarray(rotated_frames_AB)
    print("===ROTATED FRAMES GROUP AB SHAPE===\n", np.shape(rotated_frames_AB))

    distances_AB = get_distances(groupAB, rotated_frames_AB, 2)
    distances_C = get_distances(groupC, rotated_frames_C, 0)

    print("===DISTANCES GROUP AB===\n", distances_AB)
    print("=================================")
    print("===DISTANCES GROUP C===\n", distances_C)
