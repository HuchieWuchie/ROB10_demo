# SOURCE - https://realpython.com/k-means-clustering-python/
# SOURCE - https://scikit-learn.org/stable/modules/clustering.html#k-means

import os
import numpy as np
import scipy.optimize
from scipy.spatial.transform import Rotation as R

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

def k_means_clustering(observations, number_of_clusters):
    centroids = np.zeros((number_of_clusters, 4))
    previous_centroids = np.zeros((number_of_clusters, 4))
    labels = np.zeros(len(observations))
    for i in range(number_of_clusters):
            centroids[i, :] = get_random_quaternion()
    tolerance = 0.001
    max_iterations = 50
    iteration = 0
    while True:
        if iteration < max_iterations:
            labels = label_observations(observations, centroids)
            previous_centroids = centroids
            centroids, error = get_new_centroids(observations, labels, number_of_clusters)
            #print("===ERROR===")
            #print(error)

            centroids_diff = compare_centroids(previous_centroids, centroids)
            all_diff_smaller_than_tolerance = True
            for diff in centroids_diff:
                if diff > tolerance:
                    all_diff_smaller_than_tolerance = False

            if all_diff_smaller_than_tolerance:
                print("Converged")
                #print(centroids_diff)
                sum_of_errors = np.sum(error)
                print(sum_of_errors)
                converged = True
                #break
                return sum_of_errors, converged
            else:
                print("Iteration " + str(iteration))
                converged = False
                
                #print("Next iteration")
                #print(centroids_diff)
        else:
            break

        iteration = iteration + 1
    """
    while CURRENT_POSE - PREVIOUS_POSE > TOLERANCE:
        LOOP THROUGH OBSERVATIONS, CALCULTE THE DISTANCE TO EACH CLUSTER
        LABEL EACH ORIENTATION WITH THE SMALLEST DISTANCE
        GET THE CURRENT POSE
    """
    #print(random_centroids)

def label_observations(observations, centroids):
    distances = np.zeros(len(centroids))
    labels = np.zeros(len(observations))
    for i in range(0, len(observations)):
        for j in range(0, len(centroids)):
            distances[j] = get_distance(centroids[j], observations[i])
            #if j == 1:
                #print("===DISTANCES===")
                #print(distances)
        min_distance_index = np.argmin(distances)
        #index = distances(min_distance)
        labels[i] = min_distance_index
    #print("===LABELS===")
    #print(labels)
    return labels

def get_distance(s, r):
    return min( np.linalg.norm(s-r), np.linalg.norm(s+r) )

def get_new_centroids(observations, labels, number_of_clusters):
    centroids = np.zeros((number_of_clusters,4))
    error = np.zeros(number_of_clusters)
    nth_centroid = [[] for x in range(number_of_clusters)]
    for i in range(len(labels)):
        nth_centroid[int(labels[i])].append(observations[i])
    #print(nth_centroid[0])

    for i in range(number_of_clusters):
        nth_error = 0
        centroids[i] =find_solution(np.asarray(nth_centroid[i]))
        for observation in nth_centroid[i]:
            nth_error = nth_error + get_distance(observation, centroids[i])**2
        error[i] = nth_error


    #print("===CENTROIDS===")
    #print(centroids)
    return centroids, error

def objective_function(solution, observations):
    distances = []
    for observation in observations:
        distance = get_distance(solution, observation)
        distances.append(distance)
    distances = np.asarray(distances)
    return distances.sum()

def find_solution(observations):
    min_sum = None
    min_sum_solution = None
    for i in range(0, 50):
        initial_guess = get_random_quaternion()
        xopt = scipy.optimize.minimize(objective_function, x0 = initial_guess, method='Nelder-Mead', args=(observations, ))
        # NORMALIZE THE SOLUTION
        solution_mag = np.linalg.norm(xopt.x)
        solution = xopt.x/solution_mag

        if i == 0:
            min_sum_solution = solution
            min_sum = objective_function(solution, observations)
        else:
            if np.array_equal(min_sum_solution, solution):
                continue
            else:
                sum = objective_function(solution, observations)
                if sum < min_sum:
                    min_sum_solution = solution
                    min_sum = sum
                else:
                    continue

    return min_sum_solution

def compare_centroids(previous_centroids, current_centroids):
    centroids_diff = []
    for i in range(len(previous_centroids)):
        #inv = R.from_quat(previous_centroids[i]).inv().as_quat()
        #diff = np.linalg.norm(current_centroids[i]*inv)
        diff = get_distance(current_centroids[i], previous_centroids[i])
        centroids_diff.append(diff)
        #print("===PREVIOUS CENTROIDS===")
        #print(previous_centroids)
        #print("===CURRENT CENTROIDS===")
        #print(current_centroids)
        #print("===DIFF===")
        #print(diff)
    return centroids_diff


if __name__ == '__main__':
    number_of_clusters = 1
    min_error =None
    mean_orientations = get_the_mean_orientations()
    #print(mean_orientations)
    while number_of_clusters < 7:
        for i in range(10):
            error, converged = k_means_clustering(mean_orientations, number_of_clusters)
            if i == 0:
                min_error = error
            else:
                min_error = min(error, min_error)
        number_of_clusters = number_of_clusters + 1
        print("===MINIMUM RECORDER ERROR===")
        print(min_error)
