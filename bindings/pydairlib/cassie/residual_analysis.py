import sys
import numpy as np
import matplotlib.pyplot as plt
import pydairlib
import scipy.linalg
import scipy.io
from scipy.spatial.transform import Rotation
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import kinematic
from pydrake.multibody.tree import JacobianWrtVariable

class CassieSystem():
    def __init__(self):
        # Initialize for drake
        self.builder = DiagramBuilder()
        self.drake_sim_dt = 5e-5
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        addCassieMultibody(self.plant, self.scene_graph, True, 
                            "examples/Cassie/urdf/cassie_v2.urdf", True, True)
        self.plant.Finalize()
        self.world = self.plant.world_frame()
        self.base_frame = self.plant.GetBodyByName("pelvis").body_frame()
        self.context = self.plant.CreateDefaultContext()
        
        left_rod_on_thigh = LeftRodOnThigh(self.plant)
        left_rod_on_heel = LeftRodOnHeel(self.plant)
        self.left_loop = kinematic.DistanceEvaluator(
                        self.plant, left_rod_on_heel[0], left_rod_on_heel[1], left_rod_on_thigh[0],
                        left_rod_on_thigh[1], 0.5012)
        right_rod_on_thigh = RightRodOnThigh(self.plant)
        right_rod_on_heel = RightRodOnHeel(self.plant)
        self.right_loop = kinematic.DistanceEvaluator(
                        self.plant, right_rod_on_heel[0], right_rod_on_heel[1], right_rod_on_thigh[0],
                        right_rod_on_thigh[1], 0.5012)

    def calc_vdot(self, q, v, lambda_c_gt):
        """
            The unknown term are \dot v, contact force \lambda_c_active and constraints forces \lambda_h.

            M * \dot v + bias + Cv + Kq == Bu + gravity + J_c^T*\lambda_c + J_h^T*\lambda_h     (1)
            \dot J_h * v + J_h * \dot v == 0    (2)
            \dot J_c_active * v + J_c_active * \dot v == 0 (3)

            ==>

            [M, -J_c^T, -J_h^T      [ \dot v             [ Bu + gravity - bias - Cv -Kq
            J_h, 0, 0           *     \lambda_c     =      -\dot J_h * v
            J_c_active, 0, 0          \lambda_h            -\dot J_c_active * v 
            ]                       ]                     ]
        """
        # Set configuration
        self.plant.SetPositionsAndVelocities(self.context, np.hstack((q,v)))

        # Get mass matrix
        M = self.plant.CalcMassMatrix(self.context)

        # Get bias term
        bias = self.plant.CalcBiasTerm(self.context)

        # Get the B matrix
        B = self.plant.MakeActuationMatrix()

        # Get the gravity term
        gravity = self.plant.CalcGravityGeneralizedForces(self.context)

        # Get the J_h term
        J_h = np.zeros((2, self.nv))
        J_h[0, :] = self.left_loop.EvalFullJacobian(self.context)
        J_h[1, :] = self.right_loop.EvalFullJacobian(self.context)

        # Get the JdotTimesV term
        J_h_dot_times_v = np.zeros(2,)
        J_h_dot_times_v[0] = self.left_loop.EvalFullJacobianDotTimesV(self.context)
        J_h_dot_times_v[1] = self.right_loop.EvalFullJacobianDotTimesV(self.context)

        # Get the J_c_active term
        # For simplicity test, now directly give the lambda_c ground truth from simulation to determine if contact happen
        J_c_active = None; num_contact = 0; 
        left_front_index = None; left_rear_index = None;
        right_front_index = None; right_rear_index = None;
        if np.linalg.norm(lambda_c_gt[:3]) > 0:
            point_on_foot, foot_frame = LeftToeFront(self.plant)
            J_c_left_front = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            J_c_active = J_c_left_front
            left_front_index = num_contact
            num_contact += 3
        if np.linalg.norm(lambda_c_gt[3:6]) > 0:
            point_on_foot, foot_frame = LeftToeRear(self.plant)
            J_c_left_rear = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            if J_c_active is None:
                J_c_active = J_c_left_rear
            else:
                J_c_active = np.hstack((J_c_active, J_c_left_rear))
            left_rear_index = num_contact
            num_contact += 3
        if np.linalg.norm(lambda_c_gt[6:9]) > 0:
            point_on_foot, foot_frame = RightToeFront(self.plant)
            J_c_right_front = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            if J_c_active is None:
                J_c_active = J_c_right_front
            else:
                J_c_active = np.hstack((J_c_active, J_c_right_front))
            right_front_index = num_contact
            num_contact += 3
        if np.linalg.norm(lambda_c_gt[9:12]) > 0:
            point_on_foot, foot_frame = RightToeRear(self.plant)
            J_c_right_rear = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            if J_c_active is None:
                J_c_active = J_c_right_rear
            else:
                J_c_active = np.hstack((J_c_active, J_c_right_rear))
            right_rear_index = num_contact
            num_contact += 3

def main():
    pass

if __name__ == "__main__":
    main()