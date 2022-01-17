import numpy as np
import typing

# dairlib imports
from pydairlib.multibody import makeNameToPositionsMap, \
    makeNameToVelocitiesMap

# drake imports
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.multibody.inverse_kinematics import \
    AddUnitQuaternionConstraintOnPlant, ComPositionConstraint, \
    PositionConstraint
from pydrake.multibody.optimization import CentroidalMomentumConstraint
from pydrake.autodiffutils import AutoDiffXd


def nimp():
    raise NotImplementedError("implement this function before calling")

def DerivativeConstraintFunction(q0, q1, v0, v1, plant, context):
    nimp()

class CassieLipIk:
    def __init__(self, plant, context):
        self.prog = MathematicalProgram()
        self.plant = plant
        self.context = context
        self.plant_ad = plant.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()
        self.stance_foot = None
        self.swing_foot = None
        self.dt = 0
        self.N = 0
        self.qq = None
        self.vv = None
        self.ll = None
        self.c_i = None
        self.c_f = None

    def MakeCenterOfMassPositionConstraint(self, c_i, c_f):
        com_constraint = ComPositionConstraint(
            plant=self.plant,
            model_instances=None,
            expressed_frame=self.plant.world_frame(),
            plant_context=self.context)
        self.prog.AddConstraint(
            com_constraint, np.concatenate((self.qq[0], self.c_i)))
        self.prog.AddConstraint(
            com_constraint, np.concatenate((self.qq[-1], self.c_f)))
        self.prog.AddLinearEqualityConstraint(np.identity(3), c_i, self.c_i)
        self.prog.AddLinearEqualityConstraint(np.identity(3), c_f, self.c_f)

    def MakeZeroAngularMomentumConstraint(self):
        centroidal_constraint = CentroidalMomentumConstraint(
            plant=self.plant_ad,
            model_instances=None,
            plant_context=self.context_ad,
            angular_only=True)

        for i in range(self.N):
            self.prog.AddConstraint(
                centroidal_constraint,
                np.concatenate((self.qq[i], self.vv[i], self.ll[i])))
            self.prog.AddLinearEqualityConstraint(
                np.identity(3), np.zeros(self.ll[i].shape), self.ll[i])


    def MakeUnitQuaternionConstraint(self):
        for i in range(self.N):
            AddUnitQuaternionConstraintOnPlant(
                self.plant, self.qq[i], self.prog)

    def MakeStanceFootPositionConstraint(self, stance_pos):
        stance_pos_constraint = PositionConstraint(
            self.plant, self.plant.world_frame(), stance_pos, stance_pos,
            self.stance_foot['frame'], self.stance_foot['pt'], self.context)
        for i in range(self.N):
            self.prog.AddConstraint(stance_pos_constraint, self.qq[i])

    def MakeDynamicsConstraint(self):
        nimp()

    def MakeFinalSwingFootPositionConstraint(self, final_stance_pos):
        swing_pos_constraint = PositionConstraint(
            self.plant, self.plant.world_frame(), final_stance_pos,
            final_stance_pos, self.swing_foot['frame'], self.swing_foot['pt'],
            self.context)
        self.prog.AddConstraint(swing_pos_constraint, self.qq[0])

    def SetStanceFoot(self, frame, pt):
        self.stance_foot = {'frame': frame, "pt": pt}

    def SetSwingFoot(self, frame, pt):
        self.swing_foot = {'frame': frame, "pt": pt}

    def GetNeutralPoint(self, x_i):
        nimp()

    def Solve(self, v_des, y_offset, z_com, step_duration, num_knots=10):
        assert(self.swing_foot is not None)
        assert(self.stance_foot is not None)

        self.N = num_knots
        self.dt = step_duration / num_knots
        self.qq = np.zeros((self.N, self.nq), dtype="object")
        self.vv = np.zeros((self.N, self.nv), dtype="object")
        self.ll = np.zeros((self.N, 3), dtype="object")
        self.c_i = self.prog.NewContinuousVariables(3, "c_i")
        self.c_f = self.prog.NewContinuousVariables(3, "c_f")

        for i in range(self.N):
            self.qq[i] = self.prog.NewContinuousVariables(self.nq, "q" + str(i))
            self.vv[i] = self.prog.NewContinuousVariables(self.nv, "v" + str(i))
            self.ll[i] = self.prog.NewContinuousVariables(3, "L" + str(i))

        x_com = v_des * step_duration / 2.0
        c_i = np.array([-x_com, 0, z_com])
        c_f = np.array([x_com, 0, z_com])

        stance_pos = np.array([0.0, y_offset, 0.0])
        next_stance_pos = np.array([2*x_com, -y_offset, 0.0])

        # Add IK constraints
        self.MakeUnitQuaternionConstraint()
        self.MakeCenterOfMassPositionConstraint(c_i, c_f)
        self.MakeZeroAngularMomentumConstraint()
        self.MakeStanceFootPositionConstraint(stance_pos)
        self.MakeFinalSwingFootPositionConstraint(next_stance_pos)

