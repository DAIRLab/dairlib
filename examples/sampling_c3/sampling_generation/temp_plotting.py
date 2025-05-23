import numpy as np
import matplotlib.pyplot as plt

original = np.loadtxt('examples/sampling_c3/sampling_generation/original_t.csv', delimiter=',')
points = np.loadtxt('examples/sampling_c3/sampling_generation/sampled_points.csv', delimiter=',')
plt.scatter(original[:, 0], original[:, 1], color='red', label='Original Graph')
plt.scatter(points[:, 0], points[:, 1], color='blue', label='Sampled Points')
plt.axis('equal')
plt.show()