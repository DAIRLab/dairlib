import sys
import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from scipy.interpolate import CubicSpline

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.multibody

def main():
  # Default filename for the example
  filename = FindResourceOrThrow('../dairlib_data/goldilocks_models/find_models/robot_1/dircon_trajectory')
  if len(sys.argv) == 2:
    filename = sys.argv[1]
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)

  """
  States, inputs, forces trajectories
  """

  # Indexing
  x_idx_start = 7
  x_idx_end = 19

  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]
  force_knot = dircon_traj.GetForceSamples(0)
  t_coll = dircon_traj.GetCollocationForceBreaks(0)
  force_coll = dircon_traj.GetCollocationForceSamples(0)

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  input_traj = dircon_traj.ReconstructInputTrajectory()
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes
  force_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0), dircon_traj.GetForceSamples(0))
  force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes
  force_c_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetCollocationForceBreaks(0), dircon_traj.GetCollocationForceSamples(0))
  force_c_datatypes = dircon_traj.GetTrajectory("collocation_force_vars0").datatypes

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  # state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  state_samples = np.zeros((n_points, x_idx_end - x_idx_start))
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
  force_c_samples = np.zeros((n_points, force_c_traj.value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[x_idx_start:x_idx_end, 0]
    input_samples[i] = input_traj.value(t[i])[:, 0]
    force_samples[i] = force_traj.value(t[i])[:, 0]
    force_c_samples[i] = force_c_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories
  plt.figure("state trajectory")
  plt.plot(t, state_samples)
  plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
  plt.legend(state_datatypes[x_idx_start:x_idx_end])

  plt.figure("input trajectory")
  plt.plot(t, input_samples)
  plt.legend(input_datatypes)

  # plt.figure("force trajectory")
  # plt.plot(t, force_samples)
  # plt.legend(force_datatypes)
  #
  # plt.figure("collocation force trajectory")
  # plt.plot(t, force_c_samples)
  # plt.legend(force_c_datatypes)

  # plt.figure("force at knots and collocation pts")
  # plt.plot(t_knot, force_knot.T)
  # plt.plot(t_coll, force_coll.T, 'ko', markersize=4)
  # plt.legend(force_datatypes + force_c_datatypes)

  # Reconstruct force at both collocation and knots
  t_knot_col = np.zeros((t_knot.shape[0] + t_coll.shape[0], 1))
  force_knot_col = np.zeros((force_knot.shape[0],t_knot_col.shape[0]))
  force_knot_col[:, 0] = force_knot[:, 0]
  for i in range(1, t_knot.shape[0]):
    t_knot_col[2*i - 1] = t_coll[i-1]
    t_knot_col[2*i] = t_knot[i]
    force_knot_col[:, 2*i - 1] = force_coll[:, i-1]
    force_knot_col[:, 2*i] = force_knot[:, i]
  plt.figure("force at knots and collocation pts")
  plt.plot(t_knot_col, force_knot_col.T)
  plt.plot(t_coll, force_coll.T, 'ko', markersize=2)
  plt.legend(force_datatypes + force_c_datatypes)

  """
  Center of mass trajectories
  """
  # Build MBP
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"))
  plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # MBP params
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  # Conext and world
  context = plant.CreateDefaultContext()
  world = plant.world_frame()

  # Get data at knots
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)
  xdot_knot = dircon_traj.GetStateDerivativeSamples(0)

  com_at_knot = np.zeros((3, t_knot.shape[0]))
  comdot_at_knot = np.zeros((3, t_knot.shape[0]))
  comddot_at_knot = np.zeros((3, t_knot.shape[0]))
  for i in range(t_knot.shape[0]):
    xi = x_knot[:, i]
    plant.SetPositionsAndVelocities(context, xi)
    com_at_knot[:, i] = plant.CalcCenterOfMassPosition(context)
    J = plant.CalcJacobianCenterOfMassTranslationalVelocity(context, JacobianWrtVariable.kV, world, world)
    comdot_at_knot[:, i] = J @ x_knot[nq:, i]
    JdotV_i = plant.CalcBiasCenterOfMassTranslationalAcceleration(context, JacobianWrtVariable.kV, world, world)
    comddot_at_knot[:, i] = J @ xdot_knot[nq:, i] + JdotV_i

  com_at_coll = np.zeros((3, t_knot.shape[0] - 1))
  comdot_at_coll = np.zeros((3, t_knot.shape[0] - 1))
  comddot_at_coll = np.zeros((3, t_knot.shape[0] - 1))
  for i in range(t_knot.shape[0] - 1):
    # Get x and xdot at collocation points
    x_cs = CubicSpline(np.squeeze(t_knot[i: i+2]), x_knot[:, i:i+2].T, bc_type=((1, xdot_knot[:, i].T), (1, xdot_knot[:, i+1].T)))
    x_col = x_cs(np.average(t_knot[i: i+2]))
    xdot_col = x_cs(1, np.average(t_knot[i: i+2]))
    # Compute com, comdot, comddot
    plant.SetPositionsAndVelocities(context, x_col)
    com_at_coll[:, i] = plant.CalcCenterOfMassPosition(context)
    J = plant.CalcJacobianCenterOfMassTranslationalVelocity(context, JacobianWrtVariable.kV, world, world)
    comdot_at_coll[:, i] = J @ x_col[nq:]
    JdotV_i = plant.CalcBiasCenterOfMassTranslationalAcceleration(context, JacobianWrtVariable.kV, world, world)
    comddot_at_coll[:, i] = J @ xdot_col[nq:] + JdotV_i

  # plt.figure("com traj at knot pts")
  # plt.plot(t_knot, com_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("comdot traj at knot pts")
  # plt.plot(t_knot, comdot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("comddot traj at knot pts")
  # plt.plot(t_knot, comddot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])

  plt.figure("com traj at knot and coll pts")
  plt.plot(t_knot, com_at_knot.T)
  plt.gca().set_prop_cycle(None)  # reset color cycle
  plt.plot(t_coll, com_at_coll.T, 'o', markersize=2)
  plt.plot(t_knot, com_at_knot.T, 'ko', markersize=2)
  plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
  plt.figure("comdot traj at knot and coll pts")
  plt.plot(t_knot, comdot_at_knot.T)
  plt.gca().set_prop_cycle(None)  # reset color cycle
  plt.plot(t_coll, comdot_at_coll.T, 'o', markersize=2)
  plt.plot(t_knot, comdot_at_knot.T, 'ko', markersize=2)
  plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
  plt.figure("comddot traj at knot and coll pts")
  plt.plot(t_knot, comddot_at_knot.T)
  plt.gca().set_prop_cycle(None)  # reset color cycle
  plt.plot(t_coll, comddot_at_coll.T, 'o', markersize=2)
  plt.plot(t_knot, comddot_at_knot.T, 'ko', markersize=2)
  plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])


  plt.show()


if __name__ == "__main__":
  main()