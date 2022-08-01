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
    
    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input = \
        get_log_data(log,                                       # log
                     franka_channels,                           # lcm channels
                     -1,                                        # end time
                     mbp_plots.load_default_franka_channels,    # processing callback
                     plant, "FRANKA_OUTPUT", "FRANKA_INPUT")    # processing callback arguments

    print('Finished processing log - making plots')

    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)

    ''' Plot Positions '''
    mbp_plots.plot_joint_positions(robot_output, pos_names,
                                   0, t_x_slice)

    ''' Plot Velocities '''
    mbp_plots.plot_joint_velocities(robot_output, vel_names,
                                    0, t_x_slice)
    ''' Plot Measured Efforts '''
    mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
    
    ''' Plot Desired Efforts '''
    mbp_plots.plot_desired_efforts(robot_output, act_names, t_x_slice)

    plt.show()


if __name__ == '__main__':
    main()
