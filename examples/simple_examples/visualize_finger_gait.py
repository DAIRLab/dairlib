import numpy as np
from dairlib import lcmt_robot_output
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import FindResourceOrThrow
from scipy.spatial.transform import Rotation as R
from pydairlib.multibody import *
from pydairlib.systems import RobotOutputSender
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource
from pydairlib.multibody import MultiposeVisualizer
from pydrake.math import RigidTransform
from pydrake.lcm import DrakeLcm
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.lcm import LcmScopeSystem




if __name__ == '__main__':
  lcm = DrakeLcm()
  t_x = np.load('examples/simple_examples/data/alp_visual/finger_gaiting/' + 'time_finger_gait.npy')
  x_traj = np.load('examples/simple_examples/data/alp_visual/finger_gaiting/' + 'state_finger_gait.npy')
  u_traj = np.load('examples/simple_examples/data/alp_visual/finger_gaiting/' + 'input_finger_gait.npy')
  lambda_raw_traj = np.load('examples/simple_examples/data/alp_visual/finger_gaiting/' + 'contact_finger_gait.npy')

  #t_x = np.load('examples/simple_examples/data2/' + 'time_pivoting.npy')
  #x_traj = np.load('examples/simple_examples/data2/' + 'state_pivoting.npy')
  #u_traj = np.load('examples/simple_examples/data2/' + 'input_pivoting.npy')
  #lambda_raw_traj = np.load('examples/simple_examples/data2/' + 'contact_pivoting.npy')

  # x_traj = np.load('x.npy')

  # t_x = np.zeros(10)
  # t_x = np.arange(0, 10, 0.1)
  # q_traj = np.zeros((t_x.shape[0], 5))

  builder = DiagramBuilder()
  # plant = MultibodyPlant(1e-5)
  # scene_graph = builder.AddSystem(SceneGraph())
  # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-5)
  plant = MultibodyPlant(1e-4)
  scene_graph = builder.AddSystem(SceneGraph())
  plant_id = plant.RegisterAsSourceForSceneGraph(scene_graph)


  Parser(plant).AddModelFromFile(FindResourceOrThrow('examples/simple_examples/urdf/finger_pen.urdf'))
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   RigidTransform())
  plant.Finalize()

  name_map = makeNameToPositionsMap(plant)
  for name in name_map:
    print(name + ': ' + str(name_map[name]))
  print(plant.num_positions()) 

  # plant_id = plant.get_source_id()

  x_traj *= 0.05

  # pp_traj, _ = self.make_two_cube_piecewise_polynomial(cube_data, sim_data)
  q_traj = np.zeros((5, t_x.shape[0]))
  q_traj[0] = x_traj[0]
  q_traj[1] = x_traj[2]
  q_traj[2] = x_traj[2]
  q_traj[3] = x_traj[4]
  q_traj[4] = x_traj[4]


  qv_traj = np.zeros((10, t_x.shape[0]))
  qv_traj[0] = x_traj[0]
  qv_traj[1] = x_traj[2]
  qv_traj[2] = x_traj[2]
  qv_traj[3] = x_traj[4]
  qv_traj[4] = x_traj[4]
  qv_traj[5] = x_traj[1]
  qv_traj[6] = x_traj[3]
  qv_traj[7] = x_traj[3]
  qv_traj[8] = x_traj[5]
  qv_traj[9] = x_traj[5]

  scaling_force = 8

  lambda_traj = np.zeros((12, t_x.shape[0]-1))
  lambda_traj[2] = (lambda_raw_traj[4] - lambda_raw_traj[5])/scaling_force
  lambda_traj[0] = u_traj[3]/scaling_force #normal direction

  lambda_traj[5] = (lambda_raw_traj[1] - lambda_raw_traj[2])/scaling_force
  lambda_traj[3] = u_traj[2]/scaling_force

  lambda_traj[8] = (lambda_raw_traj[4] - lambda_raw_traj[5])/scaling_force
  lambda_traj[6] = -u_traj[3]/scaling_force

  lambda_traj[11] = (lambda_raw_traj[1] - lambda_raw_traj[2])/scaling_force
  lambda_traj[9] = -u_traj[2]/scaling_force

  print(q_traj)
  #q_traj *= 0.05
  pp_traj = PiecewisePolynomial.FirstOrderHold(t_x, q_traj)

  # Wire up the simulation
  traj_source = builder.AddSystem(TrajectorySource(pp_traj))
  q_to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
  builder.Connect(traj_source.get_output_port(), q_to_pose.get_input_port())
  builder.Connect(q_to_pose.get_output_port(), scene_graph.get_source_pose_port(plant_id))

  # create and connect force publisher
  lambda_pp_traj = TrajectorySource(PiecewisePolynomial.FirstOrderHold(t_x[0:-1], lambda_traj))
  lambda_source = builder.AddSystem(lambda_pp_traj)
  scope, publisher = LcmScopeSystem.AddToBuilder(
      builder=builder,
      lcm=lcm,
      signal=lambda_source.get_output_port(),
      channel="FORCES",
      publish_period=0.0)

  # create and connect state publisher
  qv_pp_traj = TrajectorySource(PiecewisePolynomial.FirstOrderHold(t_x, qv_traj))
  qv_source = builder.AddSystem(qv_pp_traj)
  output_sender = builder.AddSystem(RobotOutputSender(plant, False))
  builder.Connect(qv_source.get_output_port(),
                  output_sender.get_input_port_state());
  state_publisher = builder.AddSystem(
      LcmPublisherSystem.Make(
          channel="STATE",
          lcm_type=lcmt_robot_output,
          lcm=lcm,
          publish_period=0.0,
          use_cpp_serializer=True))
  builder.Connect(output_sender.get_output_port(), state_publisher.get_input_port(0))

  DrakeVisualizer.AddToBuilder(builder, scene_graph)
  diagram = builder.Build()
  sim = Simulator(diagram)
  sim.set_publish_every_time_step(True)

  realtime_rate = 1
  t_end = t_x[-1]
  sim.set_target_realtime_rate(realtime_rate)

  # while(True):
  sim.get_mutable_context().SetTime(t_x[0])
  sim.Initialize()
  sim.AdvanceTo(t_end)