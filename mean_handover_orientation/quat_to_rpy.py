# https://rock-learning.github.io/pytransform3d/index.html
# https://rock-learning.github.io/pytransform3d/_modules/pytransform3d/transform_manager.html#TransformManager
import numpy as np
from scipy.spatial.transform import Rotation as R
import os



if __name__ == '__main__':
    with open('/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_handover_orientations.txt') as f:
        lines = f.readlines()

    output_path = "/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_orientations_rpy.txt"
    with open(output_path, 'w') as f:
        f.truncate()

    mean_orientations_quat = np.zeros((12,4))
    mean_orientations_rpy =np.zeros((12,3))
    idx = 0

    for line in lines:
        line = line.replace("[","")
        line = line.replace("]","")
        temp = line.split()

        mean_orientation = temp[1:5]
        for i in range(0,4):
            mean_orientation[i] = float(mean_orientation[i])
        mean_orientations_quat[idx, :] = mean_orientation

        rpy = R.from_quat(mean_orientation).as_rotvec()
        rpy = np.degrees(rpy)
        mean_orientations_rpy[idx, :] = rpy
        idx = idx + 1
        rpy = rpy.tolist()
        #print(rpy)

        with open(output_path, 'a') as f:
            f.write(temp[0] + " " + str(rpy) + "\n")
        print(i)
        print(mean_orientations_quat)
        print(mean_orientations_rpy)

    np.savetxt("/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_orientations_quat.csv", mean_orientations_quat, delimiter=",")
    np.savetxt("/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_orientations_rpy.csv", mean_orientations_rpy, delimiter=",")
