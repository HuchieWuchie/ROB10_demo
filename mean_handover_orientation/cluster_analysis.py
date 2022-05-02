# SOURCE - https://realpython.com/k-means-clustering-python/
# SOURCE - https://scikit-learn.org/stable/modules/clustering.html#k-means

import os
import numpy as np

def get_the_mean_orientations():
    with open('/home/daniel/iiwa_ws/src/ROB10/mean_handover_orientation/mean_handover_orientations.txt') as f:
        lines = f.readlines()

    mean_orientations = np.zeros((len(lines), 4))
    for i in range(0, len(lines)):
        line = lines[i]
        line = line.replace("[","")
        line = line.replace("]","")
        temp = line.split()
        temp.pop(0)
        map(float, temp)
        mean_orientations[i, :] = temp

    return mean_orientations

def get_random_quaternion():
    q = np.zeros(4)
    q[0] = np.random.uniform(low=-1.0, high=1.0) #q_x
    q[1] = np.random.uniform(low=-1.0, high=1.0) #q_y
    q[2] = np.random.uniform(low=-1.0, high=1.0) #q_z
    q[3] = np.random.uniform(low=-1.0, high=1.0) #q_w
    # normalize quaternion
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    return q

def k_means_clustering(dataset, number_of_clusters):
    random_centroids = np.zeros((number_of_cluster, 4))
    for i in range(0, number_of_cluster):
            random_centroids[i, :] = get_random_quaternion()
    tolerance = 1e-6
    while CURRENT_POSE - PREVIOUS_POSE > TOLERANCE:
        LOOP THROUGH OBSERVATIONS, CALCULTE THE DISTANCE TO EACH CLUSTER
        LABEL EACH ORIENTATION WITH THE SMALLEST DISTANCE
        GET THE CURRENT POSE 
    #print(random_centroids)

if __name__ == '__main__':
    number_of_clusters = 2
    mean_orientations = get_the_mean_orientations()
    print(mean_orientations)
    k_means_clustering(mean_orientations, number_of_clusters)
