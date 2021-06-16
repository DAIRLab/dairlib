# -*- coding: utf-8 -*-
"""
LineCoSpar for exoskeleton setting
"""

import numpy as np
import itertools
from scipy.stats import norm
from scipy.optimize import minimize
import scipy.io as io
import sys
import time
import matplotlib.pyplot as plt
from matplotlib import rcParams
#from opt_utils import *

# Unpack command line arguments:
# num_samples = int(sys.argv[1])     # Number of times to sample from the posterior
num_samples = 20
save_folder = '/home/yangwill/Documents/research/projects/cassie/hardware/gain_tuning/' # folder where all results are saved'
hyperparam_file = 'gain_tuning_params'
data_file = save_folder + 'gain_tuning_data'
output_file = 'gain_tuning_output.txt'
print(sys.argv)
# iter_count = sys.argv[1]

#Load best point and visited points
best_point_file = save_folder + 'best_point.npy'
visited_points_file = save_folder + 'visited_points.npy'
pts_file = save_folder + 'prev_points_to_sample.npy'
data_matrix_file = save_folder + 'data_matrix.npy' # all prefeences

# Load hyperparameter input arguments:
inputs = io.loadmat(save_folder + hyperparam_file)
GP_noise_var =  inputs['GP_noise_var'][0][0]
preference_noise = inputs['preference_noise'][0][0] # Preference noise hyperparameter = 0.005
discretization = inputs['discretization'][0] #probably ~10 points?
state_dim = inputs['dims'][0][0]
lbs = inputs['lower_bounds'][0]
ubs = inputs['upper_bounds'][0]

""" Helper functions """
import numpy as np
import itertools
from scipy.stats import norm
from scipy.optimize import minimize
import scipy.io as io
import sys
import time
import os
import glob
import matplotlib.pyplot as plt
from matplotlib import rcParams

def reset(save_folder):
  files = glob.glob('save_folder/*')
  for f in files:
    os.remove(f)


def plot_progress(points_to_sample, posterior_model, reward_models, num_samples,save_folder):
  timestamp = str(time.time())[-3:]
  # plt.rcParams["font.family"] = "Arial"

  # Unpack model posterior:
  post_mean = posterior_model['mean']
  cov_evecs = np.real(posterior_model['cov_evecs'])
  cov_evals = posterior_model['cov_evals']

  # Construct posterior covariance matrix:
  post_cov = cov_evecs @ np.diag(cov_evals) @ np.linalg.inv(cov_evecs)

  # Posterior standard deviation at each point:
  post_stdev = np.sqrt(np.diag(post_cov))

  rcParams.update({'font.size': 40})

  fig = plt.figure(figsize = (9, 18))

  x_values = np.arange(len(points_to_sample))

  # Plot posterior mean and standard deviation:
  plt.plot(x_values, post_mean, color = 'blue', linewidth = 5)
  plt.fill_between(x_values, post_mean - 2*post_stdev,
                   post_mean + 2*post_stdev, alpha = 0.3, color = 'blue')


  # Plot posterior samples:
  for j in range(num_samples):

    reward_model = reward_models[j]

    plt.plot(x_values, reward_model, color = 'green',
             linestyle = '--', linewidth = 3)

  plt.xlabel('Visited actions')

  post_file = save_folder + "/post_file.npy"
  """load data"""
  try:
    post_dict = np.load(post_file,allow_pickle=True).item()
  except FileNotFoundError:
    print('no post file')
    post_dict = {}
    post_dict['means'] = []
    post_dict['stds'] = []
    post_dict['samples'] = []

  """save data"""
  post_dict['means'].append(post_mean)
  post_dict['stds'].append(post_stdev)
  post_dict['samples'].append(reward_models)
  np.save(post_file, post_dict)


  plt.legend(['Posterior','Posterior samples'], loc = 'upper right')

  pref_num = 1
  plt.savefig('posterior.png')

  plt.savefig(save_folder + '/posterior_%s.png' %timestamp)



def get_random_direction(state_dim):
  """
  creates a random directional vector in state_dim dimensions
  """
  direction = np.random.normal(size=state_dim)
  direction /= np.linalg.norm(direction)
  # direction *= ub - lb  # scale direction with parameter ranges
  return direction

def is_valid_point(pt, lbs, ubs):
  if any(pt[i] < lbs[i] for i in range(len(pt))):
    return False
  if any(pt[i] > ubs[i] for i in range(len(pt))):
    return False
  return True


def get_prior_randomdir(save_folder, state_dim, lbs, ubs, dists, queried_points, best_point, signal_variance = 0.0001):
  points_to_sample = []
  dists = np.asarray(dists)
  direction = get_random_direction(state_dim)
  num_new_pts = 0
  #get points along line in negative direction
  n = 1
  while 1:
    new_point = best_point + n*dists*direction
    if is_valid_point(new_point, lbs, ubs):
      points_to_sample.append(new_point)
    else:
      break
    n += 1

  #get points along line in positive direction
  n = 1
  while 1:
    new_point = best_point - n*dists*direction
    if is_valid_point(new_point, lbs, ubs):
      points_to_sample.append(new_point)
    else:
      break
    n += 1
  num_new_pts = len(points_to_sample)
  if len(points_to_sample) == 0:
    return get_prior_randomdir(save_folder, state_dim, lbs, ubs, dists, queried_points, best_point, signal_variance)

  prior_dim = len(points_to_sample)
  if np.array(queried_points).size != 0:
    prior_dim += len(queried_points)
    queried_points = list(queried_points)
    queried_points.extend(points_to_sample)
    points_to_sample = queried_points

  lengthscales = [0.15] * state_dim          # TODO 0.03 in matlab setup. Larger values = smoother reward function
  GP_prior_cov = signal_variance * np.ones((prior_dim, prior_dim))
  GP_noise_var = 1e-5        # GP model noise--need at least a very small
  # number to ensure that the covariance matrix
  #  is invertible.

  for i in range(prior_dim):

    pt1 = points_to_sample[i]

    for j in range(prior_dim):

      pt2 = points_to_sample[j]

      for dim in range(state_dim):

        lengthscale = lengthscales[dim]

        if lengthscale > 0:
          GP_prior_cov[i, j] *= np.exp(-0.5 * ((pt2[dim] - pt1[dim]) / lengthscale)**2)

        elif lengthscale == 0 and pt1[dim] != pt2[dim]:

          GP_prior_cov[i, j] = 0

  GP_prior_cov += GP_noise_var * np.eye(prior_dim)

  subspace_file = save_folder + "/subspace_file.npy"
  """load data"""
  try:
    subspace_dict = np.load(subspace_file,allow_pickle=True).item()
  except FileNotFoundError:
    print("no subspace file")
    subspace_dict = {}
    subspace_dict['directions'] = []
    subspace_dict['best_points'] = []
    subspace_dict['points_to_sample'] = []
    subspace_dict['num_new_points'] = []

  """save data"""
  subspace_dict['directions'].append(direction)
  subspace_dict['best_points'].append(best_point)
  subspace_dict['points_to_sample'].append(points_to_sample)
  subspace_dict['num_new_points'].append(num_new_pts)
  np.save(subspace_file, subspace_dict)
  return best_point, GP_prior_cov, points_to_sample


def advance(posterior_model, num_samples, cov_scale = 1):
  """
  Draw a specified number of samples from the preference GP Bayesian model
  posterior.

  Inputs:
      1) posterior_model: this is the model posterior, represented as a
         dictionary of the form {'mean': post_mean, 'cov_evecs': evecs,
         'cov_evals': evals}; post_mean is the posterior mean, a length-n
         NumPy array in which n is the number of points over which the
         posterior is to be sampled. cov_evecs is an n-by-n NumPy array in
         which each column is an eigenvector of the posterior covariance,
         and evals is a length-n array of the eigenvalues of the posterior
         covariance.

      2) num_samples: the number of samples to draw from the posterior; a
         positive integer.

      3) cov_scale: parameter between 0 and 1; this is multiplied to the
         posterior standard deviation, so values closer to zero result in
         less exploration.

  Outputs:
      1) A num_samples-length NumPy array, in which each element is the index
         of a sample.
      2) A num_samples-length list, in which each entry is a sampled reward
         function. Each reward function sample is a length-n vector (see
         above for definition of n).

  """

  samples = np.empty(num_samples)    # To store the sampled actions

  # Unpack model posterior:
  mean = posterior_model['mean']
  cov_evecs = posterior_model['cov_evecs']
  cov_evals = posterior_model['cov_evals']

  num_features = len(mean)

  R_models = []       # To store the sampled reward functions

  # Draw the samples:
  for i in range(num_samples):

    # Sample reward function from GP model posterior:
    X = np.random.normal(size = num_features)
    R = mean + cov_scale * cov_evecs @ np.diag(np.sqrt(cov_evals)) @ X

    R = np.real(R)

    samples[i] = np.argmax(R) # Find where reward function is maximized

    R_models.append(R)        # Store sampled reward function

  return samples.astype(int), R_models


def feedback(data, labels, GP_prior_cov_inv, preference_noise, cov_scale = 1,
             r_init = []):
  """
  Function for updating the GP preference model given data.

  Inputs:
      1) data: num_data_points-by-data_point_dimensionality NumPy array
      2) labels: num_data_points NumPy array (all elements should be zeros
         and ones)
      3) GP_prior_cov_inv: n-by-n NumPy array, where n is the number of
         points over which the posterior is to be sampled
      4) preference_noise: positive scalar parameter. Higher values indicate
         larger amounts of noise in the expert preferences.
      5) (Optional) cov_scale: parameter between 0 and 1; this is multiplied
         to the posterior standard deviation when sampling in the advance
         function, so values closer to zero result in less exploration.
      6) (Optional) initial guess for convex optimization; length-n NumPy
         array when specified.

  Output: the updated model posterior, represented as a dictionary of the
         form {'mean': post_mean, 'cov_evecs': evecs, 'cov_evals': evals};
         post_mean is the posterior mean, a length-n NumPy array in which n
         is the number of points over which the posterior is to be sampled.
         cov_evecs is an n-by-n NumPy array in which each column is an
         eigenvector of the posterior covariance, and evals is a length-n
         array of the eigenvalues of the posterior covariance.

  """
  num_features = GP_prior_cov_inv.shape[0]

  # Solve convex optimization problem to obtain the posterior mean reward
  # vector via Laplace approximation:
  if r_init == []:
    r_init = np.zeros(num_features)    # Initial guess

  res = minimize(preference_GP_objective, r_init, args = (data, labels,
                                                          GP_prior_cov_inv, preference_noise), method='L-BFGS-B',
                 jac=preference_GP_gradient)

  # The posterior mean is the solution to the optimization problem:
  post_mean = res.x

  if cov_scale > 0: # Calculate eigenvectors/eigenvalues of covariance matrix

    # Obtain inverse of posterior covariance approximation by evaluating the
    # objective function's Hessian at the posterior mean estimate:
    post_cov_inverse = preference_GP_hessian(post_mean, data, labels,
                                             GP_prior_cov_inv, preference_noise)

    # Calculate the eigenvectors and eigenvalues of the inverse posterior
    # covariance matrix:
    evals, evecs = np.linalg.eigh(post_cov_inverse)

    # Invert the eigenvalues to get the eigenvalues corresponding to the
    # covariance matrix:
    evals = 1 / evals

  else:   # cov_scale = 0; evecs/evals not used in advance function.

    evecs = np.eye(num_features)
    evals = np.zeros(num_features)

  # Return the model posterior:
  best_idx = np.argmax(post_mean)

  return {'mean': post_mean, 'cov_evecs': evecs, 'cov_evals': evals, 'best_idx': best_idx}


def preference_GP_objective(f, data, labels, GP_prior_cov_inv, preference_noise):
  """
  Evaluate the optimization objective function for finding the posterior
  mean of the GP preference model (at a given point); the posterior mean is
  the minimum of this (convex) objective function.

  Inputs:
      1) f: the "point" at which to evaluate the objective function. This is
         a length-n vector, where n is the number of points over which the
         posterior is to be sampled.
      2)-5): same as the descriptions in the feedback function.

  Output: the objective function evaluated at the given point (f).
  """


  obj = 0.5 * f @ GP_prior_cov_inv @ f

  num_samples = data.shape[0]

  for i in range(num_samples):   # Go through each pair of data points

    data_pts = data[i, :].astype(int)   # Data points queried in this sample
    label = labels[i]

    if data_pts[0] == data_pts[1] or label == 0.5:
      continue

    label = int(label)

    z = (f[data_pts[label]] - f[data_pts[1 - label]]) / preference_noise
    obj -= np.log(sigmoid(z))

  return obj


def preference_GP_gradient(f, data, labels, GP_prior_cov_inv, preference_noise):
  """
  Evaluate the gradient of the optimization objective function for finding
  the posterior mean of the GP preference model (at a given point).

  Inputs:
      1) f: the "point" at which to evaluate the gradient. This is a length-n
         vector, where n is the number of points over which the posterior
         is to be sampled.
      2)-5): same as the descriptions in the feedback function.

  Output: the objective function's gradient evaluated at the given point (f).
  """

  grad = GP_prior_cov_inv @ f    # Initialize to 1st term of gradient

  num_samples = data.shape[0]

  for i in range(num_samples):   # Go through each pair of data points

    data_pts = data[i, :].astype(int)   # Data points queried in this sample
    if labels[i] < 0.5:
      label = 0
    else:
      label = 1
    # label = int(labels[i])
    # print(label)
    if data_pts[0] == data_pts[1] or label == 0.5:
      continue

    s_pos = data_pts[label]
    s_neg = data_pts[1 - label]

    z = (f[s_pos] - f[s_neg]) / preference_noise

    value = (sigmoid_der(z) / sigmoid(z)) / preference_noise

    grad[s_pos] -= value
    grad[s_neg] += value

  return grad

def preference_GP_hessian(f, data, labels, GP_prior_cov_inv, preference_noise):
  """
  Evaluate the Hessian matrix of the optimization objective function for
  finding the posterior mean of the GP preference model (at a given point).

  Inputs:
      1) f: the "point" at which to evaluate the Hessian. This is
         a length-n vector, where n is the number of points over which the
         posterior is to be sampled.
      2)-5): same as the descriptions in the feedback function.

  Output: the objective function's Hessian matrix evaluated at the given
          point (f).
  """

  num_samples = data.shape[0]

  Lambda = np.zeros(GP_prior_cov_inv.shape)

  for i in range(num_samples):   # Go through each pair of data points

    data_pts = data[i, :].astype(int)  # Data points queried in this sample
    label = int(labels[i])

    if data_pts[0] == data_pts[1] or label == 0.5:
      continue

    s_pos = data_pts[label]
    s_neg = data_pts[1 - label]

    z = (f[s_pos] - f[s_neg]) / preference_noise

    sigm = sigmoid(z)
    # print(type((sigmoid_der / sigm)**2))
    value = (-sigmoid_2nd_der(z) / sigm + (sigmoid_der(z) / sigm)**2) / (preference_noise**2)

    Lambda[s_pos, s_pos] += value
    Lambda[s_neg, s_neg] += value
    Lambda[s_pos, s_neg] -= value
    Lambda[s_neg, s_pos] -= value

  return GP_prior_cov_inv + Lambda


def sigmoid(x):
  """
  Evaluates the sigmoid function at the specified value.
  Input: x = any scalar
  Output: the sigmoid function evaluated at x.
  """

  return 1 / (1 + np.exp(-x))

def sigmoid_der(x):
  """
  Evaluates the sigmoid function's derivative at the specified value.
  Input: x = any scalar
  Output: the sigmoid function's derivative evaluated at x.
  """

  return np.exp(-x) / (1 + np.exp(-x))**2

def sigmoid_2nd_der(x):
  """
  Evaluates the sigmoid function's 2nd derivative at the specified value.
  Input: x = any scalar
  Output: the sigmoid function's 2nd derivative evaluated at x.
  """

  return (-np.exp(-x) + np.exp(-2 * x)) / (1 + np.exp(-x))**3

# Unpack observed preference data:
def add_pref_to_matrix(pt1, pt2, preference, mat,visited_point_dict):
  mat = list(mat)
  print('index of second pt')
  print(visited_point_dict[tuple(pt2)])
  mat.append([visited_point_dict[tuple(pt1)],visited_point_dict[tuple(pt2)], int(preference)])
  return np.array(mat)

def get_coactive_point(coactive_feedback):
  old_pt = np.array([float(coactive_feedback[i]) for i in range(state_dim)])
  ind = state_dim
  big = coactive_feedback[ind][0]
  ind += 1
  higher  =  coactive_feedback[ind][0]
  ind += 1
  dimension = int(coactive_feedback[ind][0])
  print(type(dimension))
  print(dimension)
  # 0 if little / 1 if a lot; 0 if shorter/1 if longer;
  new_pt = np.copy(old_pt)
  increment = 0.1 * (ubs[dimension] - lbs[dimension])

  if not higher:
    increment *= -1
  if big:
    increment *= 2

  new_pt[dimension] = min(ubs[dimension],max(old_pt[dimension] + increment, lbs[dimension]))
  return new_pt
  # print(visited_point_dict.values())
  # print(len(visited_points))
  # #add to visited points
  # if tuple(new_pt) not in visited_point_dict:
  #     visited_points = list(visited_points)
  #     count = len(visited_points)
  #     visited_points.append(tuple(new_pt))
  #     visited_point_dict[tuple(new_pt)] = count
  #     visited_points = np.array(visited_points)
  #     print("after adding coactive")
  #     print(visited_point_dict.values())
  #     print(len(visited_points))

  # return add_pref_to_matrix(old_pt, new_pt, 1, mat,visited_point_dict), visited_points,visited_point_dict


if __name__ == '__main__':

  """load data"""
  try:
    prev_best_point = np.load(best_point_file)
    print(prev_best_point)
  except FileNotFoundError:
    print('defaulting to default best point')
    prev_best_point = [np.random.uniform(lbs[i], ubs[i]) for i in range(state_dim)]

  try:
    prev_pts = np.load(pts_file)
  except FileNotFoundError:
    prev_pts = np.asarray([])

  try:
    data_matrix_dict = np.load(data_matrix_file,allow_pickle='true').item()
    #data_matrix_
    print(type(data_matrix_dict))
    data_matrix = data_matrix_dict['dat']
  except FileNotFoundError:
    print('no data matrix')
    data_matrix_dict = {}; #Maegan
    data_matrix_dict['dat'] = np.asarray([])
    data_matrix_dict['coact'] = []
    data_matrix = np.asarray([]) #Maegan
  try:
    visited_points = np.load(visited_points_file)
    to_tup = list(visited_points)
    for i in range(len(to_tup)):
      to_tup[i] = tuple(visited_points[i])
    visited_point_dict = dict(zip(to_tup,np.arange(len(visited_points))))
  except FileNotFoundError:
    print('no visited points file')
    visited_points = []
    visited_point_dict = {}

  data = io.loadmat(data_file)
  try:
    last_tried_action = np.asarray(data['last_action'][0])
  except:
    print('first time')
    last_tried_action = np.asarray([-1,-1,-1,-1,-1,-1])
  X = np.asarray(data['X'])


  """add preferences (either real or coactive)"""
  if np.shape(X)[0] == state_dim:
    label = data['y'][0]
    data_matrix = add_pref_to_matrix(X[:,0],X[:,1],label,data_matrix,visited_point_dict)
    data_matrix_dict['coact'].append(0)

  # coactive = data['C']
  # if np.shape(coactive)[0] == 9:
  #   new_coactive_point = get_coactive_point(coactive)
  #   old_pt = np.array([float(coactive[i]) for i in range(state_dim)])
  #   if tuple(new_coactive_point) not in visited_point_dict:
  #
  #     visited_points = list(visited_points)
  #     count = len(visited_points)
  #     visited_points.append(tuple(new_coactive_point))
  #     visited_point_dict[tuple(new_coactive_point)] = count
  #     visited_points = np.array(visited_points)
  #     print("after adding coactive")
  #     print(visited_point_dict.values())
  #     print(len(visited_points))
  #
  #   data_matrix = add_pref_to_matrix(old_pt,new_coactive_point,1,data_matrix,visited_point_dict)
  #   # data_matrix,visited_points,visited_point_dict = add_coactive_feedback_to_matrix(coactive,data_matrix,visited_points,visited_point_dict)
  #   data_matrix_dict['coact'].append(1)
  # print(data_matrix)
  data_matrix_dict['dat'] = data_matrix

  """add new subspace"""
  best_point,GP_prior_cov,points_to_sample = get_prior_randomdir(save_folder, state_dim, lbs, ubs, discretization, visited_points, prev_best_point)

  GP_prior_cov_inv = np.linalg.inv(GP_prior_cov)

  """Update the Gaussian process preference model"""
  if np.size(data_matrix) != 0:
    posterior_model = feedback(data_matrix[:,:2], data_matrix[:,2], GP_prior_cov_inv,
                               preference_noise)
  else:
    posterior_model = feedback(data_matrix, data_matrix, GP_prior_cov_inv,
                               preference_noise)
  prev_best_point = points_to_sample[posterior_model['best_idx']]

  """Sample new points at which to query for a preference"""
  sampled_point_idxs, R_models = advance(posterior_model, num_samples)

  # Obtain coordinate points corresponding to these indices
  sampled_points = np.empty((num_samples, state_dim))
  for j in range(num_samples):
    sampled_point_idx = sampled_point_idxs[j]
    sampled_points[j, :] = points_to_sample[sampled_point_idx]


  prev_num_visited_points = len(visited_points)
  count = prev_num_visited_points

  print(count)

  # Add sampled points to visited points.
  visited_points = list(visited_points)
  for idx in sampled_point_idxs:
    if idx >= prev_num_visited_points: #it's a point in the new subspace
      tuple_point = tuple(points_to_sample[idx])
      if tuple_point not in visited_point_dict: #if it hasn't been visited before, add it
        print("adding to visited points:")
        print(tuple_point)
        visited_point_dict[tuple_point] = count
        count += 1
        visited_points.append(tuple_point)

  visited_points = np.array(visited_points)
  print(np.shape(sampled_points))
  print(np.shape(visited_points))
  np.save(best_point_file, prev_best_point)
  if (len(sampled_points) == 2 and not np.array_equal(np.asarray(sampled_points[0]),np.asarray(sampled_points[1]))) or not np.array_equal(np.asarray(sampled_points[0]),np.asarray(last_tried_action)):
    # print(data_matrix_dict)

    plot_progress(points_to_sample, posterior_model, R_models,num_samples,save_folder)

    # Save output file with information about the sampled points:
    io.savemat(output_file, {'sampled_points_idxs': sampled_point_idxs,
                             'sampled_rewards': R_models,
                             'points_to_sample': points_to_sample,
                             'actions': sampled_points,
                             'post_mean': posterior_model['mean'],
                             'cov_evecs': posterior_model['cov_evecs'],
                             'cov_evals': posterior_model['cov_evals']})

    np.save(visited_points_file, visited_points)
    np.save(pts_file, points_to_sample)
    np.save(data_matrix_file, data_matrix_dict)
  else:

    subspace_file = save_folder + "/subspace_file.npy"
    subspace_dict = np.load(subspace_file,allow_pickle=True).item()
    """save data"""
    subspace_dict['directions'] = subspace_dict['directions'][:-1]
    subspace_dict['best_points'] = subspace_dict['best_points'][:-1]
    subspace_dict['points_to_sample']= subspace_dict['points_to_sample'][:-1]
    subspace_dict['num_new_points']= subspace_dict['num_new_points'][:-1]
    np.save(subspace_file, subspace_dict)