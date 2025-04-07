from pydrake.all import *
from pydairlib.franka.controllers import FrankaOSCControllerDiagram, FrankaC3ControllerDiagram, C3Options

from pydrake.common.yaml import yaml_load
from pydairlib.systems.primitives import *
from pydairlib.systems.robot_lcm_systems import *
from pydairlib.lcm.lcm_trajectory import *
import pydrake.systems.lcm as mut
from franka_env_helper_functions import *
import dairlib


def run_sim(intrinsics, gains):
  osc_params_file = "examples/franka/parameters/franka_osc_controller_params.yaml"
  c3_params_file = "examples/franka/parameters/franka_c3_controller_params.yaml"
  lcm_params_file = "examples/franka/parameters/lcm_channels_simulation.yaml"
  sim_params_file = "examples/franka/parameters/franka_sim_params.yaml"

  sim_params = yaml_load(filename=sim_params_file)
  lcm_params = yaml_load(filename=lcm_params_file)
  c3_options = load_c3_options(c3_params_file)

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

  c3_options.publish_frequency = (gains[0] * 5) + 25
  c3_options.Q = (1 + 0.5 * gains[1]) * c3_options.Q
  c3_options.R = (1 + 0.5 * gains[2]) * c3_options.R
  c3_options.G = (1 + 0.5 * gains[3]) * c3_options.G
  c3_options.U = (1 + 0.5 * gains[4]) * c3_options.U

  osc_controller = builder.AddSystem(FrankaOSCControllerDiagram(
    "examples/franka/parameters/franka_osc_controller_params.yaml",
    "examples/franka/parameters/lcm_channels_simulation.yaml", lcm))

  c3_controller = builder.AddSystem(FrankaC3ControllerDiagram(
    "examples/franka/parameters/franka_c3_controller_params.yaml", c3_options,
    "examples/franka/parameters/lcm_channels_simulation.yaml", lcm))

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

  q = np.hstack((sim_params['q_init_franka'], sim_params['q_init_tray'][sim_params['scene_index']]))
  v = np.zeros(plant.num_velocities())
  plant.SetPositions(plant_context, q)

  simulator.Initialize()
  simulator.AdvanceTo(5.0)

  sim_data = sim_state_logger.GetLog(logger_context)
  sim_states = sim_data.data()
  sim_times = sim_data.sample_times()

  # reward = compute_reward_sparse(sim_times, sim_states)
  reward = compute_reward_mpc(sim_times, sim_states)
  # print(reward)
  # print(sim_states.data().shape)
  # print(sim_states.sample_times().shape)
  return reward, sim_states


if __name__ == '__main__':
  parameters = 2 * np.random.random(5) - 1  # center around 0. [-1, 1]
  # reward, sim_states = run_sim(np.zeros(5), np.ones(5))
  reward, sim_states = run_sim(np.zeros(5), parameters)
  print(reward)
