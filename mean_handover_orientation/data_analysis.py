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

if __name__ == '__main__':
    classes = get_classes()
    print("===CLASSES===\n", classes)

    for class_id in classes:
        observations = get_observations(class_id)
        print("===OBSERVATIONS===\n", observations)
        mean_orientation = get_mean_orientation(class_id)
        print("===MEAN ORIENTATION===\n", mean_orientation)
        rotated_x = np.zeros((3, len(observations)))
        rotated_y = np.zeros((3, len(observations)))
        rotated_z = np.zeros((3, len(observations)))

        for i in range(len(observations)):
            rotated_frame = rotate_frame(observations[i])
            rotated_x[:, i] = rotated_frame[:,0]
            rotated_y[:, i] = rotated_frame[:,1]
            rotated_z[:, i] = rotated_frame[:,2]

        #print("===ROTATED X===\n", rotated_x)
        #print("===ROTATED Y===\n", rotated_y)
        #print("===ROTATED Z===\n", rotated_z)
        frame_mean_orientation = np.zeros((3,3))
        frame_mean_orientation = rotate_frame(mean_orientation)
        print("===FRAME MEAN ORIENTATION===\n", frame_mean_orientation)
        variance = np.zeros(3)
        variance[0] = get_variance(rotated_x, frame_mean_orientation[:, 0])
        variance[1] = get_variance(rotated_y, frame_mean_orientation[:, 1])
        variance[2] = get_variance(rotated_z, frame_mean_orientation[:, 2])
        print("===VARIANCE===\n", variance)
        exit(3)
