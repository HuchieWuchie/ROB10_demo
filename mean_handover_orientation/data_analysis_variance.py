import os
import numpy as np
#import scipy.optimize
from scipy.spatial.transform import Rotation as R

def fix_transformation(transformation):
    fixed_transformation = np.zeros((4,4))
    fixed_transformation[3,3] = 1
    q = R.from_matrix(transformation[:3, :3]).as_quat()
    # MAY NOT BE NECESSARY, SEEMS TO BE NORMALIZED
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    fixed_rot_mat = R.from_quat(q).as_matrix()
    fixed_transformation[:3, :3] = fixed_rot_mat
    return fixed_transformation

def get_quat_from_matrix(transformation):
    rot_mat = np.zeros((3,3))
    rot_mat = transformation[:3, :3]
    q = R.from_matrix(rot_mat).as_quat()
    # MAY NOT BE NECESSARY, SEEMS TO BE NORMALIZED
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    return q

def get_mean_orientation(class_id):
    mean_orientation = np.zeros(4)

    with open('/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_handover_orientations.txt') as f:
        lines = f.readlines()

    for line in lines:
        line = line.replace("[","")
        line = line.replace("]","")
        strings = line.split()
        if class_id == strings[0]:
            #print(temp)
            mean_orientation = strings[1:5]
            for i in range(0,4):
                mean_orientation[i] = float(mean_orientation[i])
            #print(mean_orientation)
    return mean_orientation

def get_observations(class_id):
    path = "/home/daniel/Transformations/transformations_parsed"
    root, dirs, _ = next(os.walk(path))
    for dir in dirs:
        if dir == class_id:
            _, _, files = next(os.walk(os.path.join(root,dir)))
            number_of_files = len(files)
            observations = np.zeros((number_of_files, 4))
            #print(np.shape(observations))
            for i in range(0, number_of_files):
                transformation = np.load(os.path.join(root,dir,files[i]))
                fixed_transformation = fix_transformation(transformation)
                observations[i, :] = get_quat_from_matrix(fixed_transformation)

    return observations

def rotate_frame(rotation):
    #print("===ROTATION===\n",rotation)
    rot_mat = R.from_quat(rotation).as_matrix()
    #print("===ROTATION MAT===\n", rot_mat)
    unit_frame = np.eye(3)
    #print("===UNIT FRAME===\n",unit_frame)
    rotated_frame = np.matmul(unit_frame, rot_mat)
    #print("===ROTATED FRAME===\n", rotated_frame)
    return rotated_frame

def get_classes():
    path = "/home/daniel/Transformations/transformations_parsed"
    root, dirs, _ = next(os.walk(path))
    return dirs

"""
def get_variance(samples, mean):
    print("===MEAN===\n", mean)
    sum = 0
    for i in range(len(samples)):
        #print("===SAMPLE===\n", samples[:, i])
        diff = samples[:,i]-mean
        #print("===DIFF===\n",diff)
        diff_squared = np.power(diff, 2)
        #print("===DIFF SQUARED===\n",diff_squared)
        sum = sum + diff_squared
        #exit(2)
    print("===SUM===\n", sum)
    variance = sum/len(samples)
    print("===VARIANCE===\n", variance)
    exit(2)
    return variance
"""

def get_variance(distances):
    variance = np.zeros(3)
    for i in range(3):
        mean = np.sum(distances[:,i])/len(distances)
        #print("===MEAN VARIANCE===\n", mean)
        sum = 0
        for j in range(len(distances)):
            delta = distances[j,i] - mean
            delta = delta**2
            sum = sum + delta
        variance[i] = sum/len(distances)

    return variance

def get_min_max_variance(variance):
    variance = variance.tolist()
    min_value = min(variance)
    min_index = variance.index(min_value)
    max_value = max(variance)
    max_index = variance.index(max_value)

    return min_index, max_index


if __name__ == '__main__':
    output_path = "/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/recorded_variances.txt"
    with open(output_path, 'w') as f:
        f.truncate()

    classes = get_classes()
    print("===CLASSES===\n", classes)
    axis_agreement = [[] for i in range(6)]

    for class_id in classes:
        observations = get_observations(class_id)
        #print("===OBSERVATIONS===\n", observations)
        mean_orientation = get_mean_orientation(class_id)
        #print("===MEAN ORIENTATION===\n", mean_orientation)
        rotated_x = np.zeros((3, len(observations)))
        rotated_y = np.zeros((3, len(observations)))
        rotated_z = np.zeros((3, len(observations)))

        frame_mean_orientation = np.zeros((3,3))
        frame_mean_orientation = rotate_frame(mean_orientation)
        #print("===FRAME MEAN ORIENTATION===\n", frame_mean_orientation)
        distances = []

        for i in range(len(observations)):
            rotated_frame = rotate_frame(observations[i])
            #print("===ROTATED FRAME ===\n", rotated_frame)
            delta_mat = np.zeros((3,3))
            delta_vec = np.zeros(3)
            delta_mat = frame_mean_orientation - rotated_frame
            #print("===DELTA===\n", delta_mat)
            delta_vec[0] = np.linalg.norm(delta_mat[:,0])
            delta_vec[1] = np.linalg.norm(delta_mat[:,1])
            delta_vec[2] = np.linalg.norm(delta_mat[:,2])
            distances.append(delta_vec)

        #print("===ROTATED X===\n", rotated_x)
        #print("===ROTATED Y===\n", rotated_y)
        #print("===ROTATED Z===\n", rotated_z)
        distances = np.asarray(distances)
        #print("===DISTANCES===\n", distances)
        variance = np.zeros(3)
        variance = get_variance(distances)
        #print("===VARIANCE===\n", variance)
        min_variance_axis, max_variance_axis = get_min_max_variance(variance)
        #min_max_variance_string = "min_var_axis " + str(min_variance_axis) + " max_var_axis " + str(max_variance_axis)

        if min_variance_axis == 0:
            axis_agreement[0].append(class_id)
        if min_variance_axis == 1:
            axis_agreement[1].append(class_id)
        if min_variance_axis == 2:
            axis_agreement[2].append(class_id)
        if max_variance_axis == 0:
            axis_agreement[3].append(class_id)
        if max_variance_axis == 1:
            axis_agreement[4].append(class_id)
        if max_variance_axis == 2:
            axis_agreement[5].append(class_id)

        variance_string = str(variance)
        variance_string = variance_string.replace("[","")
        variance_string = variance_string.replace("]","")
        string_to_save = class_id + " " + variance_string + " " + "\n"

        with open(output_path, 'a') as f:
            f.write(string_to_save)

    print("===MIN VARIANCE ON X AXIS===\n", axis_agreement[0])
    print("===MIN VARIANCE ON Y AXIS===\n", axis_agreement[1])
    print("===MIN VARIANCE ON Z AXIS===\n", axis_agreement[2])
    print("==============================")
    print("===MAX VARIANCE ON X AXIS===\n", axis_agreement[3])
    print("===MAX VARIANCE ON Y AXIS===\n", axis_agreement[4])
    print("===MAX VARIANCE ON Z AXIS===\n", axis_agreement[5])
