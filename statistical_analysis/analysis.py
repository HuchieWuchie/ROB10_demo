import numpy as np
from scipy.stats import wilcoxon
# https://stats.stackexchange.com/questions/308700/wilcoxon-signed-rank-test-fails-for-small-sample-size

if __name__ == '__main__':
    observations_1 = np.random.randint(1,5, size=(8,5))
    observations_2 = np.random.randint(1,5, size=(8,5))
    rows, cols = observations_1.shape

    diff = np.zeros((rows, cols))
    means = np.zeros(cols)
    stds = np.zeros(cols)
    p_values = np.zeros(cols)

    for i in range(cols):
        diff[:, i] = observations_1[:, i] - observations_2[:, i]
        means[i] = np.mean(diff[:, i])
        stds[i] = np.std(diff[:, i])
        w, p = wilcoxon(diff[:, i], mode='approx')
        p_values[i] = p


    #print(diff)
    print(means)
    print(stds)
    print(p_values)

    #print(observations_1)
    #print(observations_2)
