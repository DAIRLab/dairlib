# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.systems.all import (
    Diagram
)

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.perception_learning.cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)


def main():
    controller_params = AlipFootstepLQROptions(
        height=1.0,
        mass=30.0,
        stance_width=0.35,
        single_stance_duration=0.3,
        double_stance_duration=0.1
    )
    sim_params = CassieFootstepControllerEnvironmentOptions()
    controller = AlipFootstepLQR(controller_params)
    sim = CassieFootstepControllerEnvironment(sim_params)


if __name__ == "__main__":
    main()