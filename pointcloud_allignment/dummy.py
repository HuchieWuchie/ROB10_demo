import os
import numpy as np
import scipy.optimize
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors

def get_random_quaternion():
    q = np.zeros(4)
    q[0] = np.random.uniform(low=-1.0, high=1.0) #q_x
    q[1] = np.random.uniform(low=-1.0, high=1.0) #q_y
    q[2] = np.random.uniform(low=-1.0, high=1.0) #q_z
    q[3] = np.random.uniform(low=-1.0, high=1.0) #q_w
    # normalize quaternion MAY BE UNNECESSARY IF YOU USE quaternion_from_euler()
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    return q

def get_distance(p1, p2):
    return np.linalg.norm(p1-p2)

def get_distances(observation, source):
    return np.linalg.norm(observation-source, axis=0)

def rotate_points(quat, points):
    rows, cols = points.shape
    rot = R.from_quat(quat).as_matrix()
    rotated_pc = np.zeros((rows, 3))
    for i in range(rows):
        rot_point = np.zeros((1,3))
        rot_point = np.matmul(points[i, :], rot)
        rotated_pc[i, :] = rot_point

    return rotated_pc

def objective_function(solution, observation, source):
    #_, cols = observation.shape
    #distances = np.zeros(cols)

    # center on 0,0,0
    observation = observation - np.mean(observation, axis = 0)
    source = source - np.mean(source, axis = 0)
    rotated_source = rotate_points(solution, source)

    neigh= NearestNeighbors(n_neighbors=1)
    neigh.fit(observation)
    distances, _ = neigh.kneighbors(rotated_source, return_distance = True)
    #print(rotated_source)
    #print(rotated_source.shape)
    #INSERT NEAREST NEIGHBOUR
    #distances = get_distances(observation, rotated_source)
    return np.sum(distances)

def find_solution(observation, source):
    min_sum = None
    min_sum_solution = None

    initial_guess = get_random_quaternion()
    #print("===MINIMIZATION===")
    xopt = scipy.optimize.minimize(objective_function, x0 = initial_guess, method='Nelder-Mead', args=(observation, source, ))
    # PRINT ALL THE OUTPUT
    #print(xopt)
    # PRINT ONLY THE SOLUTION
    #print(xopt.x)

    # NORMALIZE THE SOLUTION
    solution_mag = np.linalg.norm(xopt.x)
    solution = xopt.x/solution_mag
    min_sum_solution = solution

    """
    for i in range(0, 50):
        #print("===ITERATION " + str(i) + " ===")
        #print("===INITIAL GUESS===")
        initial_guess = get_random_quaternion()
        #print(initial_guess)
        xopt = scipy.optimize.minimize(objective_function, x0 = initial_guess, method='Nelder-Mead', args=(observation, source, ))
        # PRINT ALL THE OUTPUT
        #print(xopt)
        # PRINT ONLY THE SOLUTION
        #print(xopt.x)

        # NORMALIZE THE SOLUTION
        solution_mag = np.linalg.norm(xopt.x)
        solution = xopt.x/solution_mag
        min_sum_solution = solution
        #print("===SOLUTION===")
        #print(solution)

        if i == 0:
            min_sum_solution = solution
            min_sum = objective_function(solution, observations)
        else:
            if np.array_equal(min_sum_solution, solution):
                continue
            else:
                #print("min_sum_solution != solution")
                sum = objective_function(solution, observations)
                if sum < min_sum:
                    min_sum_solution = solution
                    min_sum = sum
                else:
                    continue
        """

    return min_sum_solution

def get_random_pointcloud(number_of_points):
    pointcloud = np.random.rand(number_of_points, 3)
    #pointcloud = np.random.rand(3, number_of_points)
    return pointcloud

    return pairs

if __name__ == '__main__':
    pc_points = 10000
    pc_observation = get_random_pointcloud(pc_points)
    pc_source = get_random_pointcloud(pc_points)
    print(pc_observation)
    print("============")
    print(pc_source)
    solution = find_solution(pc_observation, pc_source)
    print(solution)
