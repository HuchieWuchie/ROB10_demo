import numpy as np
import scipy.optimize
from scipy.spatial.transform import Rotation as R

def get_quat_from_RPY():
    RPY = np.asarray([np.pi/2, 0.0, np.pi/4])
    r = R.from_rotvec(RPY)
    mat = r.as_matrix()
    q = R.from_matrix(mat).as_quat()
    # MAY NOT BE NECESSARY, SEEMS TO BE NORMALIZED
    q_mag = np.linalg.norm(q)
    q = q/q_mag
    return q

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

def get_distance(s, r):
    return min( np.linalg.norm(s-r), np.linalg.norm(s+r) )

def objective_function(initial_guess, observations):
    distances = []
    for observation in observations:
        distance = get_distance(initial_guess, observation)
        distances.append(distance)
    distances = np.asarray(distances)
    return distances.sum()

def generate_observations(number_of_observations):
    observations = np.zeros((number_of_observations,4))
    for i in range(0,number_of_observations):
        observations[i,:] = get_random_quaternion()
    np.savetxt("observations.csv", observations, delimiter=",")

def check_the_solution(solution, observations):
    solution_sum = objective_function(solution, observations)
    print("===SOLUTION SUM===")
    print(solution_sum)

    min_sum = None
    for i in range(0, 500000):
        #print("Iteration " + str(i))
        random_guess = get_random_quaternion();
        sum = objective_function(random_guess, observations)
        min_sum = min(solution_sum, sum)

    print("===MIN SUM===")
    print(min_sum)


if __name__ == '__main__':
    quat=get_quat_from_RPY()
    print(quat)
    exit(3)
    #generate_observations(15)

    observations = np.genfromtxt('observations.csv', delimiter=',')
    print("===OBSERVATIONS===")
    print(observations)

    min_sum = None
    min_sum_solution = None
    print("===MINIMIZATION===")
    for i in range(0, 50):
        print("===ITERATION " + str(i) + " ===")

        #print("===INITIAL GUESS===")
        initial_guess = get_random_quaternion()
        #print(initial_guess)

        xopt = scipy.optimize.minimize(objective_function, x0 = initial_guess, method='Nelder-Mead', args=(observations, ))

        # PRINT ALL THE OUTPUT
        #print(xopt)
        # PRINT ONLY THE SOLUTION
        #print(xopt.x)
        solution_mag = np.linalg.norm(xopt.x)
        solution = xopt.x/solution_mag
        print("===SOLUTION===")
        print(solution)
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

    print(min_sum_solution)

    #check_the_solution(solution, observations)
