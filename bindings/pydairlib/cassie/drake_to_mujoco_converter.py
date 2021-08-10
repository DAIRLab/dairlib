import numpy as np
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from scipy.spatial.transform import Rotation as R


class DrakeToMujocoConverter():

  def __init__(self, drake_sim_dt=5e-5):
    self.builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
    addCassieMultibody(self.plant, self.scene_graph, True,
                                                     "examples/Cassie/urdf/cassie_v2.urdf", False, False)
    self.plant.Finalize()
    self.context = self.plant.CreateDefaultContext()
    self.ik_solver = InverseKinematics(self.plant, self.context, with_joint_limits=True)
    self.left_thigh_rod = LeftRodOnThigh(self.plant)
    self.right_thigh_rod = RightRodOnThigh(self.plant)
    self.left_heel_rod = LeftRodOnHeel(self.plant)
    self.right_heel_rod = RightRodOnHeel(self.plant)

  def convert_to_mujoco(self, x):


    # q_full = np.zeros(35)
    # v_full = np.zeros(32)
    q = x[0:23]
    v = x[23:45]
    q_full, v_full = self.solve_IK(x)

    q_full[:10] = q[:10]
    # left achilles rod quat
    # q_full[10]  = q
    # q_full[11]  = q
    # q_full[12]  = q
    # q_full[13]  = q
    q_full[14] = q[10]
    q_full[15] = q[12]
    q_full[16] = q[13]
    q_full[17] = q[14]
    # left foot crank + left planar rod
    # q_full[18]  = q
    # q_full[19]  = q
    q_full[20] = q[11]
    q_full[21] = q[15]
    q_full[22] = q[16]
    q_full[23] = q[17]
    # right achilles rod quat
    # q_full[24]  = q
    # q_full[25]  = q
    # q_full[26]  = q
    # q_full[27]  = q
    q_full[28] = q[18]
    q_full[29] = q[20]
    q_full[30] = q[21]
    q_full[31] = q[22]
    # right foot crank + right planar rod
    # q_full[32]  = q
    # q_full[33]  = q
    q_full[34] = q[19]

    v_full[:9] = v[:9]
    rot = R.from_quat([q[6], q[3], q[4], q[5]])
    v_full[3:6] = rot.as_dcm().T @ v[3:6]
    # left achilles omega
    # v_full[9] =  v
    # v_full[10] = v
    # v_full[11] = v
    v_full[12] = v[9]
    v_full[13] = v[11]
    v_full[14] = v[12]
    v_full[15] = v[13]
    # left foot crank + left planar rod
    # v_full[16] = v
    # v_full[17] = v
    v_full[18] = v[10]
    v_full[19] = v[14]
    v_full[20] = v[15]
    v_full[21] = v[16]
    # right achilles omega
    # v_full[22] = v
    # v_full[23] = v
    # v_full[24] = v
    v_full[25] = v[17]
    v_full[26] = v[19]
    v_full[27] = v[20]
    v_full[28] = v[21]
    # right foot crank + right planar rod
    # v_full[29] = v
    # v_full[30] = v
    v_full[31] = v[18]

    # x_full = np.hstack((q_full, v_full))
    return q_full, v_full


  def solve_IK(self, x):
    self.plant.SetPositionsAndVelocities(self.context, x)

    # self.plant.CalcRelativeTransform(self.context, self.plant.world_frame())
    left_thigh = self.plant.CalcPointsPosition(self.context, self.left_thigh_rod.first, self.left_thigh_rod.second)
    q_missing = np.zeros(35)
    v_missing = np.zeros(32)

    # import pdb; pdb.set_trace()

    # left_quat =
    #
    # q_missing[10:14] = left_quat
    # q_missing[18] = left_foot_crank
    # q_missing[19] = left_plantar_rod
    # q_missing[24:28] = right_quat
    # q_missing[32] = right_foot_crank
    # q_missing[33] = right_plantar_rod
    #
    # v_missing[9:12] = left_omega
    # v_missing[16] = left_foot_crank_dot
    # v_missing[17] = left_plantar_rod_dot
    # v_missing[22:25] = right_omega
    # v_missing[29] = right_foot_crank_dot
    # v_missing[30] = right_plantar_rod_dot


    return q_missing, v_missing