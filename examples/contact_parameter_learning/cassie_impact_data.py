# This class is the central area to load and hold all the impact data from the Cassie jumping experiments

import numpy as np
import pickle
import cassie_loss_utils

class CassieImpactData():
  def __init__(self):
    self.data_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/data/'
    self.sim_data_directory = 'examples/contact_parameter_learning/cassie_sim_data/drake_optimal/'
    self.drake_params_directory = "examples/contact_parameter_learning/drake_cassie_params/"
    self.mujoco_params_directory = "examples/contact_parameter_learning/mujoco_cassie_params/"


    self.loss_func = cassie_loss_utils.CassieLoss('pos_loss_weights')

    self.start_times = {'08': 30.61,
                        '09': 30.61,
                        '10': 30.61,
                        '11': 30.62,
                        '12': 30.64,
                        '13': 30.64,
                        '14': 30.65,
                        '15': 30.64,
                        '16': 30.64,
                        '17': 30.64,
                        '20': 30.64,
                        '21': 30.64,
                        '22': 30.64,
                        '23': 30.64,
                        '24': 30.64,
                        '25': 30.64,
                        '26': 30.64,
                        '27': 30.64,
                        '28': 30.64,
                        '29': 30.64,
                        '30': 30.64,
                        '31': 30.63,
                        '32': 30.63,
                        '33': 30.63,
                        }

    # list of valid log nums from hardware trials
    self.log_nums_all = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_real = np.hstack((np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_sim = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_all = ['%0.2d' % i for i in self.log_nums_all]
    self.log_nums_real = ['%0.2d' % i for i in self.log_nums_real]


    # Data from hardware

    # load in all the time series data (state, inputs, contact forces)
    self.x_trajs_hardware = {}
    self.t_x_hardware = {}
    self.u_trajs_hardware = {}
    self.contact_forces_hardware = {}

    for log_num in self.log_nums_real:
      self.x_trajs_hardware[log_num] = np.load(self.data_directory + 'x_' + log_num + '.npy')
      self.t_x_hardware[log_num] = np.load(self.data_directory + 't_x_' + log_num + '.npy')
      self.u_trajs_hardware[log_num] = np.load(self.data_directory + 'u_' + log_num + '.npy')
      self.contact_forces_hardware[log_num] = np.load(self.data_directory + 'lambda_' + log_num + '.npy')

    # Data from sim

    # load in all the time series data (state, inputs, contact forces)
    self.x_trajs_sim = {}
    self.t_x_sim = {}
    # self.u_trajs_sim = {}
    self.contact_forces_sim = {}

    for log_num in self.log_nums_real:
      self.x_trajs_sim[log_num] = np.load(self.sim_data_directory + 'x_' + log_num + '.npy').T
      self.t_x_sim[log_num] = np.load(self.sim_data_directory + 't_x_' + log_num + '.npy')
      # self.u_trajs_sim[log_num] = np.load(self.sim_data_directory + 'u_' + log_num + '.npy')
      self.contact_forces_sim[log_num] = np.load(self.sim_data_directory + 'lambda_' + log_num + '.npy')


    # Load in optimal parameters
    with open(self.drake_params_directory + 'drake_2021_08_25_11_45_training_15000' + '.pkl', 'rb') as f:
      self.drake_contact_parameters = pickle.load(f)
    with open(self.mujoco_params_directory + 'mujoco_2021_08_25_18_53_training_15000' + '.pkl', 'rb') as f:
      self.mujoco_contact_parameters = pickle.load(f)
    with open(self.drake_params_directory + 'optimized_z_offsets.pkl', 'rb') as file:
      self.z_offsets = pickle.load(file)
    with open(self.drake_params_directory + 'optimized_vel_offsets.pkl', 'rb') as file:
      self.vel_offsets = pickle.load(file)

    # for log_num in self.log_nums_real:
    #   print('hardware: ')
    #   print(self.x_trajs_hardware[log_num].shape)
    #   print(self.t_xs_hardware[log_num].shape)
    #   print('sim: ')
    #   print(self.x_trajs_sim[log_num].shape)


  def combine_into_single_data_file(self):

    return

