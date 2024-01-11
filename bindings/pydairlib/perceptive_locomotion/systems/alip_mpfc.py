import numpy as np
from scipy.linalg import null_space
from typing import Tuple, overload

from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
    BasicVector,
    LeafSystem,
    Context,
    InputPort,
    OutputPort
)

from pydrake.solvers import (
    MathematicalProgram,
    GurobiSolver,
    OsqpSolver
)

from pydairlib.systems.footstep_planning import (
    AlipStepToStepDynamics,
    ResetDiscretization,
    AlipGaitParams,
    CalcAd,
    CalcA,
    Stance,
)

from pydairlib.perceptive_locomotion.systems.alip_lqr import AlipFootstepLQROptions


class AlipMPFC(LeafSystem):

    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

        self.solver = GurobiSolver()


        # DLQR params
        self.K = np.zeros((2, 4))
        self.S = np.zeros((4, 4))
        self.params = alip_params
        self.A, self.B = AlipStepToStepDynamics(
            self.params.height,
            self.params.mass,
            self.params.single_stance_duration,
            self.params.double_stance_duration,
            self.params.reset_discretization
        )
        self.K, self.S = DiscreteTimeLinearQuadraticRegulator(
            self.A,
            self.B,
            self.params.Q,
            self.params.R
        )

        self.period_two_orbit_premul = np.linalg.inv(np.eye(4) - self.A @ self.A)
        self.period_two_orbit_basis = self.period_two_orbit_premul @ (self.A @ self.B - self.B)
        self.period_two_orbit_orth = null_space(self.period_two_orbit_basis.T).T
        self.period_two_orbit_subspace_cost_hessian = \
            (2 * self.period_two_orbit_orth.T @ self.period_two_orbit_orth)
        self.period_two_orbit_subspace_cost_gradient_premul = \
            (self.period_two_orbit_subspace_cost_hessian @
             self.period_two_orbit_premul @ self.B)

        self.input_port_indices = {
            'desired_velocity': self.DeclareVectorInputPort(
                "vdes", 2
            ).get_index(),
            'fsm': self.DeclareVectorInputPort(
                "fsm", 1
            ).get_index(),
            'time_until_switch': self.DeclareVectorInputPort(
                "time_until_switch", 1
            ).get_index(),
            'state': self.DeclareVectorInputPort(
                "alip_state", 4
            ).get_index()
        }
        self.output_port_indices = {
            'footstep_command': self.DeclareVectorOutputPort(
                "footstep_command", 3, self.calculate_optimal_footstep
            ).get_index(),
            'x': self.DeclareVectorOutputPort(
                'x', 4, self.calc_discrete_alip_state
            ).get_index()
        }

        self.N = 3
        self.prog = MathematicalProgram()
        self.xx = [self.prog.NewContinuousVariables(4) for _ in range(self.N)]
        self.uu = [
            self.prog.NewContinuousVariables(2) for _ in range(self.N - 1)
        ]
        self.running_cost = [
            self.prog.AddQuadraticErrorCost(
                np.eye(4), np.zeros((4,)), self.xx[i]
            ) for i in range(self.N - 1)
        ]
        self.input_reg = [
            self.prog.AddQuadraticErrorCost(
                self.params.R, np.zeros((2,)), self.uu[i]
            ) for i in range(self.N - 1)
        ]
        self.terminal_cost = self.prog.AddQuadraticErrorCost(
            self.S, np.zeros((4,)), self.xx[-1]
        )
        self.initial_state_constraint = self.prog.AddLinearEqualityConstraint(
            np.eye(4), np.zeros((4,)), self.xx[0]
        )
        self.dynamics_constraints = [
            self.prog.AddLinearEqualityConstraint(
                self.A @ self.xx[i] + self.B @ self.uu[i] - self.xx[i + 1],
                np.zeros((4,))
            ) for i in range(self.N - 1)
        ]
        self.solver = OsqpSolver()

    def get_quadradic_cost_for_vdes(self, vdes: np.ndarray) -> \
        Tuple[np.ndarray, np.ndarray, np.ndarray]:
        g = self.period_two_orbit_premul @ self.B @ vdes
        y = self.period_two_orbit_orth @ g

        return self.period_two_orbit_orth.T @ self.period_two_orbit_orth, \
               -2 * self.period_two_orbit_orth.T @ y, y.T @ y

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])

    def calc_discrete_alip_state(self, context: Context,
                                 x_disc: BasicVector) -> None:
        current_alip_state = np.copy(self.EvalVectorInput(
            context,
            self.input_port_indices['state']
        ).value().ravel())

        time_until_switch = self.EvalVectorInput(
            context,
            self.input_port_indices['time_until_switch']
        ).value().ravel()[0]

        x = CalcAd(
            self.params.height, self.params.mass, time_until_switch
        ) @ current_alip_state

        x_disc.set_value(x)

    def calculate_optimal_footstep(
            self, context: Context, footstep: BasicVector) -> None:
        vdes = self.EvalVectorInput(
            context,
            self.input_port_indices['desired_velocity']
        ).value().ravel()
        fsm = self.EvalVectorInput(
            context,
            self.input_port_indices['fsm']
        ).value().ravel()[0]
        fsm = int(fsm)

        # get the reference trajectory for the current stance mode
        stance = Stance.kLeft if fsm == 0 or fsm == 3 else Stance.kRight
        xd, ud0, ud1 = self.make_period_two_orbit(stance, vdes)

        ud = [ud0, ud1]
        for i in range(self.N - 1):
            self.input_reg[i].evaluator().UpdateCoefficients(
                2 * self.params.R, -2 * ud[i % 2], 0
            )
            self.running_cost[i].evaluator().UpdateCoefficients(
                self.period_two_orbit_subspace_cost_hessian,
                self.period_two_orbit_subspace_cost_gradient_premul @ vdes
            )
        self.terminal_cost.evaluator().UpdateCoefficients(
            100 * self.period_two_orbit_subspace_cost_hessian,
            100 * self.period_two_orbit_subspace_cost_gradient_premul @ vdes
        )

        x0 = BasicVector(4)
        self.calc_discrete_alip_state(context, x0)
        self.initial_state_constraint.evaluator().UpdateCoefficients(
            np.eye(4), x0.get_value()
        )
        result = self.solver.Solve(self.prog)
        u = result.GetSolution(self.uu[0])
        footstep_command = np.zeros((3,))
        footstep_command[:2] = u
        footstep.set_value(footstep_command)

    def make_period_two_orbit(self, stance: Stance, vdes: np.ndarray) -> \
        Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
            Calculate a reference ALIP trajectory following the philosophy
            outlined in https://arxiv.org/pdf/2309.07993.pdf, section IV.D
        """
        # First get the input sequence corresponding to the desired velocity
        s = -1.0 if stance == Stance.kLeft else 1.0
        u0 = np.zeros((2,))
        u0[0] = vdes[0] * (
                self.params.single_stance_duration +
                self.params.double_stance_duration
        )
        u0[1] = s * self.params.stance_width + vdes[1] * (
                self.params.single_stance_duration +
                self.params.double_stance_duration
        )
        u1 = np.copy(u0)
        u1[1] -= 2 * s * self.params.stance_width

        # Solve for period-2 orbit, x0 = A(Ax0 + Bu0) + Bu1
        # \therefore (I - A^2)x0 = ABu0 + Bu1
        x0 = np.linalg.solve(
            np.eye(4) - self.A @ self.A,
            self.A @ self.B @ u0 + self.B @ u1
        )
        return x0, u0, u1
