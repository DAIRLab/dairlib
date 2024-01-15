import numpy as np
from scipy.linalg import null_space
from typing import Tuple, overload

from pydrake.multibody.all import MultibodyPlant

from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
    BasicVector,
    LeafSystem,
    Context,
    InputPort,
    OutputPort,
    Value
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

from pydairlib.solvers.optimization_utils import LinearBigMConstraint, \
    LinearBigMEqualityConstraint

from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet

from pydairlib.perceptive_locomotion.systems.alip_lqr import AlipFootstepLQROptions

from pydairlib.systems import OutputVector

class AlipMPFC(LeafSystem):

    def __init__(self, alip_params: AlipFootstepLQROptions, plant: MultibodyPlant):
        super().__init__()

        self.solver = GurobiSolver()
        self.plant = plant
        self.plant_context = plant.CreateDefaultContext()

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

        self.period_two_orbit_premul = None
        self.period_two_orbit_basis = None
        self.period_two_orbit_orth = None
        self.period_two_orbit_subspace_cost_hessian = None
        self.gradient_premul_period_one = None
        self.gradient_premul_period_two = None
        self.calculate_period_two_orbit_subspace()

        self.input_port_indices = {}
        self.output_port_indices = {}
        self.declare_ports()

        self.N = 3
        self.kMaxFootholds = 10
        self.prog = MathematicalProgram()
        self.xx = []
        self.pp = []
        self.mu = []
        self.make_mpc_variables()

        self.running_cost = []
        self.input_reg = []
        self.terminal_cost = None
        self.make_mpc_costs()

        self.initial_state_constraint = self.prog.AddLinearEqualityConstraint(
            np.eye(4), np.zeros((4,)), self.xx[0]
        )
        self.initial_footstep_constraint = self.prog.AddLinearEqualityConstraint(
            np.eye(2), np.zeros((2,)), self.pp[0]
        )
        self.dynamics_constraints = [
            self.prog.AddLinearEqualityConstraint(
                self.A @ self.xx[i] + self.B @ (self.pp[i+1] - self.pp[i]) - self.xx[i + 1],
                np.zeros((4,))
            ) for i in range(self.N - 1)
        ]
        self.solver = GurobiSolver()

        self.crossover_constraints = []
        self.foothold_constraints = []
        self.foothold_choice_constraint = []
        self.state_constraints = []
        self.make_input_constraints()

    def declare_ports(self):
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
            'alip_state': self.DeclareVectorInputPort(
                "alip_state", 4
            ).get_index(),
            'robot_state': self.DeclareVectorInputPort(
                "x, u, t", OutputVector(
                    self.plant.num_positions(),
                    self.plant.num_velocities(),
                    self.plant.num_actuators()
                )
            ).get_index(),
            'convex_footholds': self.DeclareAbstractInputPort(
               "safe_footholds",
               Value(ConvexPolygonSet([])),
            )
        }
        self.output_port_indices = {
            'footstep_command': self.DeclareVectorOutputPort(
                "footstep_command", 3, self.calculate_optimal_footstep
            ).get_index(),
            'x': self.DeclareVectorOutputPort(
                'x', 4, self.calc_discrete_alip_state
            ).get_index()
        }

    def calculate_period_two_orbit_subspace(self):
        self.period_two_orbit_premul = (
            np.linalg.inv(np.eye(4) - self.A @ self.A))
        self.period_two_orbit_basis = \
            self.period_two_orbit_premul @ (self.A @ self.B - self.B)
        self.period_two_orbit_orth = null_space(self.period_two_orbit_basis.T).T
        self.period_two_orbit_subspace_cost_hessian = \
            (2 * self.period_two_orbit_orth.T @ self.period_two_orbit_orth)
        self.gradient_premul_period_one = \
            -2 * (self.period_two_orbit_subspace_cost_hessian @
                  self.period_two_orbit_premul @ self.B)
        self.gradient_premul_period_two = \
            -2 * (self.period_two_orbit_subspace_cost_hessian @ self.A @
                  self.period_two_orbit_premul @ self.B)

    def make_mpc_variables(self):
        self.xx = [self.prog.NewContinuousVariables(4, f'x_{i}') for i in
                   range(self.N)]
        self.pp = [
            self.prog.NewContinuousVariables(2, f'u_{i}') for i in
            range(self.N)
        ]
        self.mu = [
            self.prog.NewBinaryVariables(self.kMaxFootholds) for _ in
            range(self.kMaxFootholds)
        ]

    def make_mpc_costs(self):
        self.running_cost = [
            self.prog.AddQuadraticErrorCost(
                np.eye(4), np.zeros((4,)), self.xx[i]
            ) for i in range(self.N - 1)
        ]
        self.input_reg = [
            self.prog.AddQuadraticErrorCost(
                np.eye(4),
                np.zeros((4,)),
                np.concatenate([self.pp[i], self.pp[i+1]])
            ) for i in range(self.N - 1)
        ]
        self.terminal_cost = self.prog.AddQuadraticErrorCost(
            self.S, np.zeros((4,)), self.xx[-1]
        )

    def make_input_constraints(self):
        self.crossover_constraints = [
            self.prog.AddLinearConstraint(
                A=np.zeros((1, 2)),
                lb=np.array([-np.inf]),
                ub=np.array([np.inf]),
                vars=np.array([self.pp[i][1], self.pp[i+1][1]])
            ) for i in range(self.N - 1)
        ]
        self.foothold_constraints = [
            [
                LinearBigMConstraint(
                    prog=self.prog,
                    A=np.ones((1, 2)),
                    b=np.zeros((1,)),
                    M=10,
                    x=self.pp[i+1],
                    z=self.mu[i][j]
                ) for j in range(self.kMaxFootholds)
            ] for i in range(self.N - 1)
        ]

        for clist in self.foothold_constraints:
            for c in clist:
                c.deactivate()

        self.foothold_choice_constraint = [
            self.prog.AddLinearEqualityConstraint(
                np.dot(self.mu[i], np.ones((self.kMaxFootholds,))) == 1
            ) for i in range(self.N - 1)
        ]

    def update_crossover_constraints(self, stance: Stance):
        s = 1.0 if stance == Stance.kLeft else -1.0
        for c in self.crossover_constraints:
            c.evaluator().UpdateCoefficients(
                new_A=np.array([[-s, s]]),
                new_lb=np.array([-np.inf]),
                new_ub=np.array([-0.04])
            )
            s *= -1.0

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
            self.input_port_indices['alip_state']
        ).value().ravel())

        time_until_switch = self.EvalVectorInput(
            context,
            self.input_port_indices['time_until_switch']
        ).value().ravel()[0]

        x = CalcAd(
            self.params.height, self.params.mass, time_until_switch
        ) @ current_alip_state

        x_disc.set_value(x)

    def update_input_reg(self, vdes: np.ndarray, stance: Stance) -> None:
        _, ud0, ud1 = self.make_period_two_orbit(stance, vdes)
        # Update costs
        ud = [ud0, ud1]
        root_Q = np.hstack([-np.eye(2), np.eye(2)])
        Q = 2 * root_Q.T @ self.params.R @ root_Q
        for i in range(self.N - 1):
            b = -2 * root_Q.T @ self.params.R @ ud[i % 2]
            self.input_reg[i].evaluator().UpdateCoefficients(Q, b, 0)

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

        self.update_input_reg(vdes, stance)

        for i in range(self.N - 1):
            linear_term = \
                self.gradient_premul_period_one if i % 2 == 0 else \
                    self.gradient_premul_period_two
            self.running_cost[i].evaluator().UpdateCoefficients(
                new_Q=self.period_two_orbit_subspace_cost_hessian,
                new_b=linear_term @ vdes,
                is_convex=True
            )

        linear_term = self.gradient_premul_period_one if self.N % 2 else (
            self.gradient_premul_period_two)
        self.terminal_cost.evaluator().UpdateCoefficients(
            new_Q=10 * self.period_two_orbit_subspace_cost_hessian,
            new_b=10 * linear_term @ vdes,
            is_convex=True
        )

        # Update constraints
        x0 = BasicVector(4)
        self.calc_discrete_alip_state(context, x0)
        self.initial_state_constraint.evaluator().UpdateCoefficients(
            np.eye(4), x0.get_value()
        )

        # TODO (@Brian-Acosta) get initial foot position from robot and
        # add foothold constraints
        self.initial_footstep_constraint.evaluator().UpdateCoefficients(
            np.eye(2), np.zeros((2,))
        )
        self.update_crossover_constraints(stance)

        # solve the MP
        result = self.solver.Solve(self.prog)
        u = result.GetSolution(self.pp[1])
        footstep_command = np.zeros((3,))
        footstep_command[:2] = u
        footstep.set_value(footstep_command)

    def get_footholds_in_stance_frame(self):
        pass

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
        # therefore (I - A^2)x0 = ABu0 + Bu1
        x0 = self.period_two_orbit_premul @ (
            self.A @ self.B @ u0 + self.B @ u1
        )
        return x0, u0, u1
