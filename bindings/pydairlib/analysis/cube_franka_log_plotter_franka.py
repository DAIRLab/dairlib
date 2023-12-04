import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np
import scipy.io
import os
from datetime import date

import dairlib
from cube_franka_process_lcm_log import get_log_data
import mbp_plotting_utils as mbp_plots
from pydairlib.common import plot_styler, plotting_utils
from pydrake.all import *
import pydairlib.common
import yaml
from examples.cube_franka.scripts.franka_logging_utils import get_most_recent_logs
from matplotlib.patches import Rectangle

import scipy.io


#DAIR_PATH = "/home/dair-manipulation/adam_ws/dairlib"
DAIR_PATH = "/home/sharanya/workspace/dairlib"
plot_dpi = 1000

def check_flag_and_save(ps, filename, format='svg', save_flag=False):
  if save_flag:
    global plot_dpi
    ps.fig.savefig(filename, format=format, dpi=plot_dpi)

def augment_data(lcm_data1, lcm_data2, t1_key, t2_key, key1, key2):
  '''
  augments data with zero order holds so that two channels can be
  compared timewise (ex. to compute tracking error, need to compare
  EE position and desired EE position which have different timestamps)

  This function implementation can certaintly be improved, but it
  is fast enough for now
  '''

  t1 = lcm_data1[t1_key]
  t2 = lcm_data2[t2_key]
  data1 = {}
  data2 = {}
  t = set()

  for i in range(t1.shape[0]):
    data1[t1[i]] = lcm_data1[key1][i]
    t.add(t1[i])
  for i in range(t2.shape[0]):
    data2[t2[i]] = lcm_data2[key2][i]
    t.add(t2[i])
  t = sorted(list(t))

  augmented_data1 = []
  augmented_data2 = []
  hold_value1 = lcm_data1[key1][0]
  hold_value2 = lcm_data2[key2][0]
  for timestamp in t:
    if timestamp in data1:
      hold_value1 = data1[timestamp]
    if timestamp in data2:
      hold_value2 = data2[timestamp]
    augmented_data1.append(hold_value1)
    augmented_data2.append(hold_value2)
  
  return np.array(t), np.array(augmented_data1), np.array(augmented_data2)


def plot_EE_position_error(t, q, x_d, plant, context, frame, pt_on_frame, ref_frame=None):
  if ref_frame is None:
    ref_frame = plant.world_frame()
  
  x = mbp_plots.make_point_positions_from_q(q, plant, context, frame, pt_on_frame, ref_frame)
  err = x_d-x

  data = {'t': t, 'err': err}
  key = 'err'
  legend = ['x', 'y', 'z']

  ps = plot_styler.PlotStyler()
  plotting_utils.make_plot(
    data,
    't',
    slice(t.shape[0]),
    [key],
    {key: slice(3)},
    {key: legend},
    {'xlabel': 'Time',
     'ylabel': 'Error [m]',
     'title': 'EE Position Error'},
    ps
  )

  return ps

def plot_EE_velocity_error(t, q, v, xdot_d, plant, context, frame, pt_on_frame, ref_frame=None):
  if ref_frame is None:
    ref_frame = plant.world_frame()
  
  xdot = mbp_plots.make_point_velocities(q, v, plant, context, frame, pt_on_frame, ref_frame)
  err = xdot_d-xdot

  data = {'t': t, 'err': err}
  key = 'err'
  legend = ['x', 'y', 'z']

  ps = plot_styler.PlotStyler()
  plotting_utils.make_plot(
    data,
    't',
    slice(t.shape[0]),
    [key],
    {key: slice(3)},
    {key: legend},
    {'xlabel': 'Time',
     'ylabel': 'Error [m]',
     'title': 'EE Velocity Error'},
    ps
  )

  return ps


def main():
    ''' Read config '''
    config_path = 'bindings/pydairlib/analysis/plot_configs/franka_default_plot.yaml'
    with open(config_path, 'r') as stream:
      config = yaml.safe_load(stream)

    global plot_dpi
    plot_dpi = config['plot_dpi']

    ''' Get the plant '''
    builder = DiagramBuilder()
    plant = MultibodyPlant(0.0)

    parser = Parser(plant)
    parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
        "examples/cube_franka/robot_properties_fingers/urdf/franka_box.urdf"))
    parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
        "examples/cube_franka/robot_properties_fingers/urdf/jack.urdf"))

    X_WI = RigidTransform.Identity()
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
    plant.Finalize()
    context = plant.CreateDefaultContext()

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    # franka_channels = \
    #   {'FRANKA_INPUT': dairlib.lcmt_robot_input,
    #    'FRANKA_INPUT_WO_G': dairlib.lcmt_robot_input,
    #    'FRANKA_OUTPUT': dairlib.lcmt_robot_output,
    #    'FRANKA_STATE_ESTIMATE': dairlib.lcmt_robot_output,
    #    'CONTROLLER_INPUT': dairlib.lcmt_c3,
    #    'VISION_OUTPUT': dairlib.lcmt_ball_position,
    #    'CAM0_OUTPUT': dairlib.lcmt_ball_position,
    #    'CAM1_OUTPUT': dairlib.lcmt_ball_position,
    #    'CAM2_OUTPUT': dairlib.lcmt_ball_position} 

    franka_channels = \
      {'FRANKA_OUTPUT': dairlib.lcmt_robot_output,
       'CONTROLLER_INPUT': dairlib.lcmt_c3} 
    
    ''' set up log directory paths '''
    if len(sys.argv) == 1:
      logdir, log_num = get_most_recent_logs()
      # print("In if statement of cube_franka_log_plotter_franka")
      # print(log_num)
    else:
      filename = sys.argv[1]
      path_components = os.path.normpath(filename).split(os.sep)
      log_num = path_components[-2]
      logdir = ''
      for comp in path_components[1:]:
        if comp == log_num:
          break
        logdir += '/{}'.format(comp)
    
    # print("log number after getting most recent log in cube_franka_log_plotter")
    print(log_num)
    ''' Read the log '''
    filename = "{}/{}/lcmlog-{}".format(logdir, log_num, log_num)
    print("Printing filename")
    print(filename)
    print("Processing {}".format(filename))
    log = lcm.EventLog(filename, "r")

    # robot_output, robot_input, c3_output, \
    # cam0_output, cam1_output, cam2_output, vision_output = \
        # get_log_data(log,                                       # log
        #              franka_channels,                           # lcm channels
        #              config['end_time'],                        # end time
        #              mbp_plots.load_default_franka_channels,    # processing callback
        #              plant, "FRANKA_STATE_ESTIMATE", "FRANKA_INPUT_WO_G",
        #              "CONTROLLER_INPUT", "CAM0_OUTPUT", "CAM1_OUTPUT",
        #              "CAM2_OUTPUT", "VISION_OUTPUT")   
    robot_output, robot_input = \
    get_log_data(log,                                       # log
                     franka_channels,                           # lcm channels
                     config['end_time'],                        # end time
                     mbp_plots.load_default_franka_channels,    # processing callback
                     plant, "FRANKA_OUTPUT", "CONTROLLER_INPUT")   
                     
    print('Finished processing log - making plots')

    if not os.path.isdir('{}/{}/figures'.format(logdir, log_num)):
      os.makedirs('{}/{}/figures'.format(logdir, log_num))
    os.chdir(DAIR_PATH)

    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)

    ''' Plot Circle '''
    if config['plot_ball_position']:
        ball_pos_names = ['base_x', 'base_y']
        qs = [pos_map[name] for name in ball_pos_names]
        qhold = robot_output['q']
    
    
        tsize = robot_output['t_x'].size
        circle2 = plt.Circle((0.55, 0), 0.1, color='b', fill=False)
    
        xxx = qhold[:,qs[0]]
        yyy = qhold[:,qs[1]]
        plt.plot(xxx,yyy)
        plt.xlim([0.4, 0.7])
        plt.ylim([-0.15, 0.15])
        plt.gca().add_patch(circle2)
    
        plt.show()
    

    # ''' Plot Line '''
    # if config['plot_ball_position']:
    #       ball_pos_names = ['base_x', 'base_y']
    #       qs = [pos_map[name] for name in ball_pos_names]
    #       qhold = robot_output['q']

    #       tsize = robot_output['t_x'].size
          
    #       # Define the two points for the straight line
    #       # Replace the below points with the start and end points defined in our param file
    #       point1 = (3, 4)
    #       point2 = (5, 6)

    #       # Create a figure and axis
    #       fig, ax = plt.subplots()

    #       # Plot the straight line
    #       ax.plot([point1[0], point2[0]], [point1[1], point2[1]], color='b', label='Line')

    #       # Plot the data
    #       xxx = qhold[:, qs[0]]
    #       yyy = qhold[:, qs[1]]
    #       ax.plot(xxx, yyy, label='Logged Data')

    #       # Set axis limits (optional and to be modified according to our start and end points)
    #       # ax.set_xlim([0.4, 0.7])
    #       # ax.set_ylim([-0.15, 0.15])

    #       # Add a legend
    #       ax.legend()

    #       # Display the plot
    #       plt.show()
    
    # '''Plot Straight Line trajectory'''
    # if config['plot_ball_position']:
    #     ball_pos_names = ['base_x', 'base_y']
    #     qs = [pos_map[name] for name in ball_pos_names]
    #     qhold = robot_output['q']
    
    
    #     tsize = robot_output['t_x'].size
    #     reference_line = plt.plot(0.55,0 ,-0.55,0 , 'ro-')
    #     # circle2 = plt.Circle((0.55, 0), 0.1, color='b', fill=False)
    
    #     xxx = qhold[:,qs[0]]
    #     yyy = qhold[:,qs[1]]
    #     plt.plot(xxx,yyy)
    #     plt.xlim([0.4, 0.7])
    #     plt.ylim([-0.15, 0.15])
    #     plt.gca().add_patch(reference_line)
    
    #     plt.show()

    # ''' Plot Rectangle '''
    # if config['plot_ball_position']:
    #     ball_pos_names = ['base_x', 'base_y']
    #     qs = [pos_map[name] for name in ball_pos_names]
    #     qhold = robot_output['q']

    #     tsize = robot_output['t_x'].size
    #     circle2 = plt.Circle((0.55, 0), 0.1, color='b', fill=False)
    #     plt.gca().add_patch(circle2)

    #     xxx = qhold[:,qs[0]]
    #     yyy = qhold[:,qs[1]]
    #     plt.plot(xxx,yyy)

    #     #plt.xlim([0.3, 0.9])
    #     #plt.ylim([-0.12, 0.12])
    #     #rect = Rectangle((0.55,-0.1),0.1,0.2,0.0,  color='b', fill = False)
    #     #plt.gca().add_patch(rect)

    #     plt.show()

    print("creating mat file")
    mdic = {"x": xxx, "y": yyy}
    scipy.io.savemat('xy_ball.mat', mdic)
    print("finished creating mat file")


    ''' Plot Joint Positions '''
    if config['plot_joint_positions']:
      franka_pos_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', \
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
      joint_positions_ps = mbp_plots.plot_positions_by_name( \
        robot_output, franka_pos_names, t_x_slice, pos_map, 'Franka Joint Positions')
      check_flag_and_save(joint_positions_ps,
        "{}/{}/figures/joint_positions{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])
    
    ''' Plot Joint Velocities '''
    if config['plot_joint_velocities']:
      franka_vel_names = ['panda_joint1dot', 'panda_joint2dot', 'panda_joint3dot', \
        'panda_joint4dot', 'panda_joint5dot', 'panda_joint6dot', 'panda_joint7dot']
      joint_velocities_ps = mbp_plots.plot_velocities_by_name( \
        robot_output, franka_vel_names, t_x_slice, vel_map, 'Franka Joint Velocities')
      check_flag_and_save(joint_velocities_ps,
        "{}/{}/figures/joint_velocities{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Measured Joint Torques '''
    if config['plot_measured_joint_efforts']:
      joint_measured_efforts_ps = mbp_plots.plot_measured_efforts( \
        robot_output, act_names, t_x_slice)
      check_flag_and_save(joint_measured_efforts_ps,
        "{}/{}/figures/joint_efforts_measured{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Desired Joint Torques '''
    if config['plot_desired_joint_efforts']:
      joint_desired_efforts_ps = mbp_plots.plot_desired_efforts( \
        robot_input, act_names, t_x_slice)
      check_flag_and_save(joint_desired_efforts_ps,
        "{}/{}/figures/joint_efforts_desired{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Ball Position '''
    if config['plot_ball_position']:
      ball_pos_names = ['base_x', 'base_y']
      ball_position_ps = mbp_plots.plot_positions_by_name( \
        robot_output, ball_pos_names, t_x_slice, pos_map, 'Ball Position')
      check_flag_and_save(ball_position_ps,
        "{}/{}/figures/ball_position{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Ball Velocity '''
    if config['plot_ball_velocity']:
      ball_vel_names = ['base_vx', 'base_vy']
      ball_velocity_ps = mbp_plots.plot_velocities_by_name( \
        robot_output, ball_vel_names, t_x_slice, vel_map, 'Ball Velocity')
      check_flag_and_save(ball_velocity_ps,
        "{}/{}/figures/ball_velocity{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Read EE offset '''
    c3_param_path = 'examples/cube_franka/parameters.yaml'
    with open(c3_param_path, 'r') as stream:
      c3_params = yaml.safe_load(stream)
    offset = c3_params['EE_offset']

    ''' Plot EE position '''
    if config['plot_EE_position']:
      frame_names = ['panda_link10']
      pts = {'panda_link10': np.array((offset[0], offset[1], offset[2]))}
      dims = {'panda_link10': [0, 1, 2]}
      EE_position_ps = mbp_plots.plot_points_positions(robot_output, t_x_slice, \
        plant, context, frame_names, pts, dims)
      check_flag_and_save(EE_position_ps,
        "{}/{}/figures/EE_positions{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot EE velocity '''
    if config['plot_EE_velocity']:
      frame_names = ['panda_link10']
      pts = {'panda_link10': np.array((offset[0], offset[1], offset[2]))}
      dims = {'panda_link10': [0, 1, 2]}
      EE_velocity_ps = mbp_plots.plot_points_velocities(robot_output, t_x_slice, \
        plant, context, frame_names, pts, dims)
      check_flag_and_save(EE_velocity_ps,
        "{}/{}/figures/EE_velocity{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot desired EE position '''
    t_c3_slice = slice(c3_output['t'].size)
    if config['plot_desired_EE_position']:
      desired_EE_position_ps = mbp_plots.plot_c3_plan(
        c3_output, 'x_d', t_c3_slice)
      check_flag_and_save(desired_EE_position_ps,
        "{}/{}/figures/desired_EE_position{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])
    
    ''' Plot desired EE velocity '''
    if config['plot_desired_EE_velocity']:
      desired_EE_velocity_ps = mbp_plots.plot_c3_plan(
        c3_output, 'xdot_d', t_c3_slice)
      check_flag_and_save(desired_EE_velocity_ps,
        "{}/{}/figures/desired_EE_velocity{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])
    
    ''' Plot desired EE velocity '''
    if config['plot_desired_EE_forces']:
      desired_EE_force_ps = mbp_plots.plot_c3_plan(
        c3_output, 'f_d', t_c3_slice)
      check_flag_and_save(desired_EE_force_ps,
        "{}/{}/figures/desired_EE_force{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])
    
    ''' Plot C3 solve times '''
    if config['plot_desired_EE_forces']:
      solve_times_ps = mbp_plots.plot_c3_plan(
        c3_output, 'solve_times', t_c3_slice)
      check_flag_and_save(solve_times_ps,
        "{}/{}/figures/c3_solve_times{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    #st = act_map['solve_times']

    #qhold = c3_output['solve_times']
    #print(qhold)

    print(np.size( c3_output['solve_times']))
    print(np.size( c3_output['t'] ))

    ttt = c3_output['t']
    sss = c3_output['solve_times']

    #plt.plot(ttt, sss)

    #plt.show()

    print("creating mat file for solve times")
    mdic = {"t": ttt, "s": sss}
    scipy.io.savemat('solve_times.mat', mdic)

    ''' Plot EE error '''
    if config['plot_EE_error']:
      # plot position error
      pt_on_frame = np.array((offset[0], offset[1], offset[2]))
      frame = plant.GetBodyByName('panda_link10').body_frame()
      t, q_aug, x_aug = augment_data(robot_output, c3_output, 't_x', 't', 'q', 'x_d')
      EE_position_error_ps = plot_EE_position_error(t, q_aug, x_aug, plant, context, frame, pt_on_frame)
      check_flag_and_save(EE_position_error_ps,
        "{}/{}/figures/EE_position_error{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

      t, v_aug, xdot_aug = augment_data(robot_output, c3_output, 't_x', 't', 'v', 'xdot_d')
      EE_velocity_error_ps = plot_EE_velocity_error(t, q_aug, v_aug, xdot_aug, plant, context, frame, pt_on_frame)
      check_flag_and_save(EE_velocity_error_ps,
        "{}/{}/figures/EE_velocity_error{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])


    ''' Plot Unfiltered Vision Position Estimate '''
    if config['plot_unfiltered_ball_position']:
      t_slice = slice(vision_output['t'].size)
      ball_position_ps = mbp_plots.plot_ball_position(vision_output, 'xyz', t_slice)
      check_flag_and_save(ball_position_ps,
        "{}/{}/figures/unfilted_ball_position{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Vision Period '''
    if config['plot_vision_dt']:
      t_slice = slice(vision_output['t'].size)
      vision_period_ps = mbp_plots.plot_ball_position(vision_output, 'dt', t_slice)
      check_flag_and_save(vision_period_ps,
        "{}/{}/figures/vision_period{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot Vision Period '''
    if config['plot_num_cameras']:
      t_slice = slice(vision_output['t'].size)
      num_valid_cameras_ps = mbp_plots.plot_ball_position(vision_output, 'num_cameras_used', t_slice)
      check_flag_and_save(num_valid_cameras_ps,
        "{}/{}/figures/num_valid_cameras{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    ''' Plot specified camera channels '''
    if len(config['plot_camera_channels']) != 0:
      t_slice = slice(vision_output['t'].size)
      legend = []
      camera_outputs = []
      for channel in config['plot_camera_channels']:
        if channel.startswith("CAM"):
          cam_id = 'cam' + channel[3]
          camera_outputs.append(eval(cam_id + '_output'))
          legend.append(cam_id + '_x')
          legend.append(cam_id + '_y')
          legend.append(cam_id + '_z')
        else:
          camera_outputs.append(vision_output)
          legend.append('vision_output_x')
          legend.append('vision_output_y')
          legend.append('vision_output_z')

      camera_channels_ps = mbp_plots.plot_multiple_ball_positions(camera_outputs, t_slice, legend)
      check_flag_and_save(camera_channels_ps,
        "{}/{}/figures/camera_channels{}.svg".format(logdir, log_num, log_num),
        save_flag = config['save_plots'])

    if config['show_plots']:
      print("Finished making plots. Showing plots now.")
      plt.show()
    else:
      print("Finished making plots.")


if __name__ == '__main__':
    main()