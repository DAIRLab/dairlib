import numpy as np
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import FindResourceOrThrow
from scipy.spatial.transform import Rotation as R
from pydairlib.multibody import *
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource
from pydairlib.multibody import MultiposeVisualizer
from pydrake.math import RigidTransform





if __name__ == '__main__':

  t_x = np.load('/home/yangwill/Downloads/for_alp/' + 'time_finger_gait.npy')
  x_traj = np.load('/home/yangwill/Downloads/for_alp/' + 'state_finger_gait.npy')
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
  # import pdb; pdb.set_trace()


  # pp_traj, _ = self.make_two_cube_piecewise_polynomial(cube_data, sim_data)
  q_traj = np.zeros((5, t_x.shape[0]))
  q_traj[0] = x_traj[0]
  q_traj[1] = x_traj[2]
  q_traj[2] = x_traj[2]
  q_traj[3] = x_traj[4]
  q_traj[4] = x_traj[4]
  q_traj *= 0.05
  pp_traj = PiecewisePolynomial.FirstOrderHold(t_x, q_traj)

  # Wire up the simulation
  traj_source = builder.AddSystem(TrajectorySource(pp_traj))
  q_to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
  builder.Connect(traj_source.get_output_port(), q_to_pose.get_input_port())
  builder.Connect(q_to_pose.get_output_port(), scene_graph.get_source_pose_port(plant_id))

  DrakeVisualizer.AddToBuilder(builder, scene_graph)
  diagram = builder.Build()
  sim = Simulator(diagram)
  sim.set_publish_every_time_step(True)

  realtime_rate = 1.0
  t_end = t_x[-1]
  sim.set_target_realtime_rate(realtime_rate)

  # while(True):
  sim.get_mutable_context().SetTime(t_x[0])
  sim.Initialize()
  sim.AdvanceTo(t_end)