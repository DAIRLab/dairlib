import numpy as np
from dataclasses import dataclass, field

from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
    LeafSystem,
    Context,
)

from pydairlib.perceptive_locomotion.controllers import (
    AlipStepToStepDynamics,
    ResetDiscretization,
    AlipGaitParams,
    Stance,
)


# parameters needed to define the discrete time ALIP model
@dataclass
class AlipFootstepLQROptions:
    height: float
    mass: float
    single_stance_duration: float
    double_stance_duration: float
    reset_discretization: ResetDiscretization = ResetDiscretization.kFOH
    Q: np.ndarray = field(default_factory=lambda: np.eye(4))
    R: np.ndarray = field(default_factory=lambda: np.eye(2))


class AlipFootstepLQR(LeafSystem):

    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

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

        self.input_ports_indices = {
            'desired_velocity': self.DeclareVectorInputPort(
                "vdes", 2
            ).get_index(),
            'fsm': self.DeclareVectorInputPort(
                "fsm", 1
            ).get_index,
            'switch_time': self.DeclareVectorInputPort(
                "time_until_switch", 1
            ).get_index(),
            'state': self.DeclareVectorInputPort(
                "alip_state", 4
            ).get_index()
        }
        self.output_port_indices = {
            'footstep_command': None #TODO
        }


    def calculate_optimal_footstep(self, context: Context) -> None:
        pass

    def get_value_estimate(self, x) -> float:
        return x.T @ self.S @ x

