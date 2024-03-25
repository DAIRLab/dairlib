from pydrake.all import *
from pydairlib.franka.controllers import FrankaOSCControllerDiagram, FrankaC3ControllerDiagram, C3Options

from pydrake.common.yaml import yaml_load
from pydairlib.systems.primitives import *
from pydairlib.systems.robot_lcm_systems import *
from pydairlib.lcm.lcm_trajectory import *
import pydrake.systems.lcm as mut

import dairlib


def main():
  osc_params_file = "examples/franka/parameters/franka_osc_controller_params.yaml"
  c3_params_file = "examples/franka/parameters/franka_c3_controller_params.yaml"
  lcm_params_file = "examples/franka/parameters/lcm_channels_simulation.yaml"
  sim_params_file = "examples/franka/parameters/franka_sim_params.yaml"

  sim_params = yaml_load(filename=sim_params_file)
  lcm_params = yaml_load(filename=lcm_params_file)
  c3_params = yaml_load(filename=c3_params_file)
  c3_options_file = c3_params['c3_options_file'][c3_params['scene_index']]
  c3_options_dict = yaml_load(filename=c3_options_file)
  c3_options = C3Options()
  c3_options.admm_iter = c3_options_dict['admm_iter']
  c3_options.rho = c3_options_dict['rho']
  c3_options.rho_scale = c3_options_dict['rho_scale']
  c3_options.num_threads = c3_options_dict['num_threads']
  c3_options.delta_option = c3_options_dict['delta_option']
  c3_options.projection_type = c3_options_dict['projection_type']
  c3_options.contact_model = c3_options_dict['contact_model']
  c3_options.warm_start = c3_options_dict['warm_start']
  c3_options.use_predicted_x0 = c3_options_dict['use_predicted_x0']
  c3_options.solve_time_filter_alpha = c3_options_dict['solve_time_filter_alpha']
  c3_options.publish_frequency = c3_options_dict['publish_frequency']
  c3_options.world_x_limits = c3_options_dict['world_x_limits']
  c3_options.world_y_limits = c3_options_dict['world_y_limits']
  c3_options.world_z_limits = c3_options_dict['world_z_limits']
  c3_options.u_horizontal_limits = c3_options_dict['u_horizontal_limits']
  c3_options.u_vertical_limits = c3_options_dict['u_vertical_limits']
  c3_options.workspace_margins = c3_options_dict['workspace_margins']
  c3_options.N = c3_options_dict['N']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.q_vector = c3_options_dict['q_vector']
  c3_options.r_vector = c3_options_dict['r_vector']
  c3_options.g_x = c3_options_dict['g_x']
  c3_options.g_gamma = c3_options_dict['g_gamma']
  c3_options.g_lambda_n = c3_options_dict['g_lambda_n']
  c3_options.g_lambda_t = c3_options_dict['g_lambda_t']
  c3_options.g_lambda = c3_options_dict['g_lambda']
  c3_options.g_u = c3_options_dict['g_u']
  c3_options.u_x = c3_options_dict['u_x']
  c3_options.u_gamma = c3_options_dict['u_gamma']
  c3_options.u_lambda_n = c3_options_dict['u_lambda_n']
  c3_options.u_lambda_t = c3_options_dict['u_lambda_t']
  c3_options.u_lambda = c3_options_dict['u_lambda']
  c3_options.u_u = c3_options_dict['u_u']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.mu = c3_options_dict['mu']
  c3_options.dt = c3_options_dict['dt']
  c3_options.solve_dt = c3_options_dict['solve_dt']
  c3_options.num_friction_directions = c3_options_dict['num_friction_directions']
  c3_options.num_contacts = c3_options_dict['num_contacts']
  c3_options.Q = c3_options_dict['w_Q'] * np.diag(np.array(c3_options_dict['q_vector']))
  c3_options.R = c3_options_dict['w_R'] * np.diag(np.array(c3_options_dict['r_vector']))
  g_vec = np.hstack((c3_options_dict['g_x'], c3_options_dict['g_lambda'], c3_options_dict['g_u']))
  u_vec = np.hstack((c3_options_dict['u_x'], c3_options_dict['u_lambda'], c3_options_dict['u_u']))
  c3_options.G = c3_options_dict['w_G'] * np.diag(g_vec)
  c3_options.U = c3_options_dict['w_U'] * np.diag(u_vec)

  lcm = DrakeLcm("udpm://239.255.76.67:7667?ttl=0")
  builder = DiagramBuilder()
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_params['dt'])

  parser = Parser(plant)
  parser.SetAutoRenaming(True)

  franka_index, = parser.AddModels(FindResourceOrThrow(sim_params['franka_model']))
  end_effector_index, = parser.AddModels(sim_params['end_effector_model'])
  tray_index, = parser.AddModels(sim_params['tray_model'])

  T_X_W = RigidTransform(RotationMatrix(), np.zeros(3))
  T_EE_W = RigidTransform(RotationMatrix(), sim_params['tool_attachment_frame'])

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName('panda_link0'), T_X_W)
  frame = plant.GetFrameByName(name='plate', model_instance=end_effector_index)
  plant.WeldFrames(plant.GetFrameByName('panda_link7'),
                   frame,
                   T_EE_W)
  left_support_index, = parser.AddModels(sim_params['left_support_model'])
  right_support_index, = parser.AddModels(sim_params['right_support_model'])
  T_S1_W = RigidTransform(RollPitchYaw(sim_params['left_support_orientation']),
                          sim_params['left_support_position'])
  T_S2_W = RigidTransform(RollPitchYaw(sim_params['right_support_orientation']),
                          sim_params['right_support_position'])
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("support", left_support_index),
                   T_S1_W)
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("support", right_support_index),
                   T_S2_W)
  plant.Finalize()



  osc_controller = builder.AddSystem(FrankaOSCControllerDiagram(
    "examples/franka/parameters/franka_osc_controller_params.yaml",
    "examples/franka/parameters/lcm_channels_simulation.yaml", lcm))

  c3_controller = builder.AddSystem(FrankaC3ControllerDiagram(
    "examples/franka/parameters/franka_c3_controller_params.yaml", c3_options,
    "examples/franka/parameters/lcm_channels_simulation.yaml", lcm))

  # c3_publish_frequency = c3_options.publish_frequency
  # placeholder_trajectory = dairlib.lcmt_timestamped_saved_traj()
  # a = Value(placeholder_trajectory)
  # discrete_time_delay = builder.AddSystem(
  #   DiscreteTimeDelay(1.0 / c3_publish_frequency, 1, Value(placeholder_trajectory)))

  # state_pub =builder.AddSystem(LcmPublisherSystem.Make(lcm_params['franka_state_channel'], lcm_type=dairlib.lcmt_robot_output,
  #   lcm=lcm, publish_triggers=TriggerType.kForced, use_cpp_serializer=True))
  # tray_state_pub =
  # builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
  #   lcm_channel_params.tray_state_channel, &lcm,
  # drake::systems::TriggerTypeSet(
  # {drake::systems::TriggerType::kForced})));

  passthrough = builder.AddSystem(SubvectorPassThrough(8, 0, 7))
  tray_state_sender = builder.AddSystem(ObjectStateSender(plant, tray_index))
  franka_state_sender = builder.AddSystem(RobotOutputSender(plant, franka_index, False, False))

  # radio_sub = builder.AddSystem(LcmSubscriberSystem.Make(channel=lcm_params['radio_channel'], lcm_type=dairlib.lcmt_radio_out, lcm=lcm, use_cpp_serializer=True))
  sim_state_logger = builder.AddSystem(
    VectorLogSink(plant.num_positions() + plant.num_velocities(), 2 * sim_params['dt']))

  builder.Connect(c3_controller.get_output_port_mpc_plan(),
                  osc_controller.get_input_port_end_effector_position())
  builder.Connect(c3_controller.get_output_port_mpc_plan(),
                  osc_controller.get_input_port_end_effector_orientation())
  builder.Connect(c3_controller.get_output_port_mpc_plan(),
                  osc_controller.get_input_port_end_effector_force())

  builder.Connect(osc_controller.get_output_port_robot_input(),
                  passthrough.get_input_port())

  builder.Connect(franka_state_sender.get_output_port(),
                  osc_controller.get_input_port_robot_state())
  builder.Connect(franka_state_sender.get_output_port(),
                  c3_controller.get_input_port_robot_state())
  builder.Connect(tray_state_sender.get_output_port(),
                  c3_controller.get_input_port_object_state())

  builder.Connect(plant.get_state_output_port(franka_index),
                  franka_state_sender.get_input_port_state())
  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender.get_input_port_state())
  builder.Connect(plant.get_state_output_port(),
                  sim_state_logger.get_input_port())
  builder.Connect(passthrough.get_output_port(),
                  plant.get_actuation_input_port())

  if sim_params['visualize_drake_sim']:
    AddDefaultVisualization(builder)

  diagram = builder.Build()
  simulator = Simulator(diagram)

  simulator.set_publish_every_time_step(True)
  simulator.set_publish_at_initialization(True)
  simulator.set_target_realtime_rate(sim_params['realtime_rate'])

  plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
  osc_controller_context = diagram.GetMutableSubsystemContext(osc_controller, simulator.get_mutable_context())
  c3_controller_context = diagram.GetMutableSubsystemContext(c3_controller, simulator.get_mutable_context())
  osc_controller.get_input_port_radio().FixValue(osc_controller_context, np.zeros(18))
  c3_controller.get_input_port_radio().FixValue(c3_controller_context, np.zeros(18))
  logger_context = diagram.GetSubsystemContext(sim_state_logger, simulator.get_context())

  q = np.hstack((sim_params['q_init_franka'], sim_params['q_init_plate'][sim_params['scene_index']]))
  v = np.zeros(plant.num_velocities())
  plant.SetPositions(plant_context, q)

  simulator.Initialize()
  simulator.AdvanceTo(5.0)

  sim_states = sim_state_logger.GetLog(logger_context)
  # print(sim_states.data().shape)
  # print(sim_states.sample_times().shape)


if __name__ == '__main__':
  main()
