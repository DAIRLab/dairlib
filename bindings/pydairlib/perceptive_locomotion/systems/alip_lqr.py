import numpy as np
from typing import Tuple, overload
from dataclasses import dataclass, field

import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
    BasicVector,
    LeafSystem,
    Context,
    InputPort,
    OutputPort
)

from pydrake.multibody.plant import MultibodyPlant

from pydairlib.systems.footstep_planning import (
    AlipStepToStepDynamics,
    MassNormalizedAlipStepToStepDynamics,
    ResetDiscretization,
    AlipGaitParams,
    CalcAd,
    CalcA,
    CalcMassNormalizedAd,
    CalcMassNormalizedA,
    Stance,
)


# parameters needed to define the discrete time ALIP model
@dataclass
class AlipFootstepLQROptions:
    height: float
    mass: float
    stance_width: float
    single_stance_duration: float
    double_stance_duration: float
    reset_discretization: ResetDiscretization = ResetDiscretization.kFOH
    Q: np.ndarray = field(default_factory=lambda: np.eye(4))
    R: np.ndarray = field(default_factory=lambda: np.eye(2))

    @staticmethod
    def calculate_default_options(
        mpfc_gains_yaml: str, plant:
        MultibodyPlant, plant_context: Context) -> "AlipFootstepLQROptions":
        with io.open(mpfc_gains_yaml, 'r') as file:
            data = load(file, Loader=Loader)

        disc = {
            'FOH': ResetDiscretization.kFOH,
            'ZOH': ResetDiscretization.kZOH,
        }

        return AlipFootstepLQROptions(
            height=data['h_des'],
            mass=plant.CalcTotalMass(plant_context),
            stance_width=data['stance_width'],
            single_stance_duration=data['ss_time'],
            double_stance_duration=data['ds_time'],
            reset_discretization=disc[data['reset_discretization_method']],
            Q=np.array(data['q']).reshape((4, 4)),
            R=np.array(data['w_footstep_reg']).reshape(3, 3)[:2, :2],
        )


def calc_collision_cost_grid(X: np.ndarray, Y: np.ndarray, ud) -> np.ndarray:
    assert (X.shape == Y.shape)

    # If in left stance, ud[1] < 0, crossover means y > 0
    sign_allowed = np.sign(ud[1])
    sign_y = np.sign(Y)
    cost = np.zeros(Y.shape)
    cost[sign_y != sign_allowed] = np.inf
    return cost


class AlipFootstepLQR(LeafSystem):

    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

        # DLQR params
        self.K = np.zeros((2, 4))
        self.S = np.zeros((4, 4))
        self.params = alip_params
        self.A, self.B = MassNormalizedAlipStepToStepDynamics(
            self.params.height,
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
            'lqr_reference': self.DeclareVectorOutputPort(
                "xd_ud[x,y]", 6, self.calc_lqr_reference
            ).get_index(),
            'x': self.DeclareVectorOutputPort(
                'x', 4, self.calc_discrete_alip_state
            ).get_index()
        }

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])

    def calc_lqr_reference(self, context: Context, xd_ud: BasicVector) -> None:
        """
            Calculates the LQR reference trajectory for an output port
        """
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

        xd, ud = self.make_lqr_reference(stance, vdes)

        output = np.zeros((6,))
        output[:4] = xd
        output[4:] = ud
        xd_ud.set_value(output)

    def calc_discrete_alip_state(self, context: Context,
                                 x_disc: BasicVector) -> None:
        current_alip_state = np.copy(self.EvalVectorInput(
            context,
            self.input_port_indices['state']
        ).value().ravel())

        # mass normalized alip state
        current_alip_state[2:] /= self.params.mass

        time_until_switch = self.EvalVectorInput(
            context,
            self.input_port_indices['time_until_switch']
        ).value().ravel()[0]

        x = CalcMassNormalizedAd(
            self.params.height, time_until_switch
        ) @ current_alip_state

        x_disc.set_value(x)

    def calculate_optimal_footstep(
            self, context: Context, footstep: BasicVector) -> None:
        """
            Calculate the optimal (LQR) footstep location.
            This is essentially (29) in https://arxiv.org/pdf/2101.09588.pdf,
            using the LQR gain instead of the deadbeat gain. It also involves
            some book-keeping to get the appropriate states and deal with
            left/right stance.
        """
        xd_ud = self.get_output_port_by_name('lqr_reference').Eval(context)
        xd = xd_ud[:4]
        ud = xd_ud[4:]
        x = self.get_output_port_by_name('x').Eval(context)

        # LQR feedback - for now assume the height of the ground is zero
        footstep_command = np.zeros((3,))
        footstep_command[:2] = ud - self.K @ (x - xd)
        footstep.set_value(footstep_command)

    def make_lqr_reference(self, stance: Stance, vdes: np.ndarray) -> \
            Tuple[np.ndarray, np.ndarray]:
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
        return x0, u0

    def get_next_value_estimate(self, x, u, xd, ud) -> float:
        return self._get_value_estimate(
            self.A @ (x - xd) + self.B @ (u - ud)
        )

    def get_next_value_estimate_for_footstep(
            self, u: np.ndarray, context: Context) -> float:
        x = self.get_output_port_by_name('x').Eval(context)
        xd_ud = self.get_output_port_by_name('lqr_reference').Eval(context)
        xd = xd_ud[:4]
        ud = xd_ud[4:]
        return self.get_next_value_estimate(x, u[:2], xd, ud)

    def _get_value_estimate(self, xe: np.ndarray) -> float:
        return xe.T @ self.S @ xe

    def get_value_estimate(self, context: Context) -> float:
        x = self.get_output_port_by_name('x').Eval(context)
        xd_ud = self.get_output_port_by_name('lqr_reference').Eval(context)
        xd = xd_ud[:4]
        return self._get_value_estimate(x - xd)