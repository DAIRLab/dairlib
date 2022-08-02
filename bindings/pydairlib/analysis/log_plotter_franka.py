import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
from process_lcm_log import get_log_data
import mbp_plotting_utils as mbp_plots
from pydrake.all import *
import pydairlib.common

import subprocess
import os
import glob
import codecs
from datetime import date


def main():
    ''' Get the plant '''
    builder = DiagramBuilder()
    plant = MultibodyPlant(0.0)

    parser = Parser(plant)
    parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
        "examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf"))
    parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
        "examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))

    X_WI = RigidTransform.Identity()
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
    plant.Finalize()

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    franka_channels = \
      {'FRANKA_INPUT': dairlib.lcmt_robot_input,
       'FRANKA_OUTPUT': dairlib.lcmt_robot_output, # SIM ouput
       'FRANKA_ROS_OUTPUT': dairlib.lcmt_robot_output} # HW output
    
    ''' set up log directory paths '''
    if len(sys.argv) == 1:
      curr_date = date.today().strftime("%m_%d_%y")
      year = date.today().strftime("%Y")
      logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)

      os.chdir(logdir)
      current_logs = sorted(glob.glob('*'))
      if current_logs:
          if current_logs[-1] == 'log_descriptions.txt':
              last_log = int(current_logs[-2])
          else:
              last_log = int(current_logs[-1])
          log_num = "{:02}".format(last_log)
      else:
          log_num = '00'
      filename = "{}/{}/lcmlog-{}".format(logdir, log_num, log_num)
    else:
      filename = sys.argv[1]
      path_components = os.path.normpath(filename).split(os.sep)
      log_num = path_components[-2]
      logdir = ''
      for comp in path_components[1:]:
        if comp == log_num:
          break
        logdir += '/{}'.format(comp)

    ''' Read the log '''
    filename = "{}/{}/lcmlog-{}".format(logdir, log_num, log_num)
    print("Processing {}".format(filename))
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input = \
        get_log_data(log,                                       # log
                     franka_channels,                           # lcm channels
                     -1,                                        # end time
                     mbp_plots.load_default_franka_channels,    # processing callback
                     plant, "FRANKA_ROS_OUTPUT", "FRANKA_INPUT")    # processing callback arguments

    print('Finished processing log - making plots')

    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)

    ''' Plot Positions '''
    joint_positions_ps = mbp_plots.plot_joint_positions(robot_output, pos_names,
                                   0, t_x_slice)

    ''' Plot Velocities '''
    joint_velocities_ps = mbp_plots.plot_joint_velocities(robot_output, vel_names,
                                    0, t_x_slice)
    ''' Plot Measured Efforts '''
    joint_measured_efforts_ps = mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
    
    ''' Plot Desired Efforts '''
    joint_desired_efforts_ps = mbp_plots.plot_desired_efforts(robot_output, act_names, t_x_slice)

    ''' Save plots as svg '''
    image_format = 'svg'
    joint_positions_ps.fig.savefig("{}/{}/joint_positions{}.svg".format(logdir, log_num, log_num), format=image_format)
    joint_velocities_ps.fig.savefig("{}/{}/joint_velocities{}.svg".format(logdir, log_num, log_num), format=image_format)
    joint_measured_efforts_ps.fig.savefig("{}/{}/joint_measured_efforts{}.svg".format(logdir, log_num, log_num), format=image_format)
    joint_desired_efforts_ps.fig.savefig("{}/{}/joint_desired_efforts{}.svg".format(logdir, log_num, log_num), format=image_format)

    plt.show()


if __name__ == '__main__':
    main()
