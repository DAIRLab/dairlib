import sys

import lcm
import numpy as np
import pathlib
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.lcm import process_lcm_log
import pydairlib.multibody
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import plot_styler
from pydairlib.lcm import lcm_trajectory


def main():
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/data/'
  ps = plot_styler.PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  builder = DiagramBuilder()
  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_wo_spr, scene_graph_wo_spr, True,
                                                   "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)
  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  # joint/actuator names
  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant_w_spr)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant_w_spr)

  #
  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log, t_lcmlog_u = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

  # Default time window values, can override
  t_start = t_u[10]
  t_end = t_u[-10]
  # Override here #
  t_start = 30.595
  t_end = t_start + 0.51
  ### Convert times to indices
  t_slice = slice(np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0])
  t_u_slice = slice(np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0])

  log_file_num = filename.split('-')[1]
  print(log_file_num)
  x = x[t_slice]
  u_meas = u_meas[t_slice]
  t_x = np.reshape(t_x[t_slice], (t_x[t_slice].shape[0], 1))
  t_u = t_x
  np.save(ps.directory + 'x_' + log_file_num, x)
  np.save(ps.directory + 't_x_' + log_file_num, t_x)
  np.save(ps.directory + 'u_' + log_file_num, u_meas)

  controller_input_traj = lcm_trajectory.Trajectory()
  controller_input_traj.traj_name = 'controller_inputs'
  controller_input_traj.time_vector = t_u
  controller_input_traj.datapoints = u_meas.transpose()
  controller_input_traj.datatypes = [''] * u_meas.shape[0]
  lcm_traj = lcm_trajectory.LcmTrajectory()
  lcm_traj.AddTrajectory('controller_inputs', controller_input_traj)
  lcm_traj.WriteToFile(ps.directory + 'u_traj_' + log_file_num)

if __name__ == '__main__':
  main()