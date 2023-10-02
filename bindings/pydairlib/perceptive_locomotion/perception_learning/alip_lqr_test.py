# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
)

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.perception_learning.cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

import numpy as np


def main():
    controller_params = AlipFootstepLQROptions(
        height=1.0,
        mass=30.0,
        stance_width=0.35,
        single_stance_duration=0.3,
        double_stance_duration=0.1,
        Q = np.eye(4),
        R = np.eye(2)
    )
    sim_params = CassieFootstepControllerEnvironmentOptions()
    controller = AlipFootstepLQR(controller_params)
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    builder = DiagramBuilder()
    builder.AddSystem(sim_env)

    desired_velocity = ConstantVectorSource(np.array([0.1, 0.1]))
    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)

    # controller give footstep command to sim_environment (i.e. cassie)
    builder.Connect(
        controller.get_output_port(),
        sim_env.get_input_port_by_name("footstep_command")
    )

    # external user assign desire velocity to controller
    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port(0)
    )

    # sim_env (cassie) returns state_feedback to controller
    builder.Connect(
        sim_env.get_output_port_by_name("fsm"),
        controller.get_input_port(1)
    )
    builder.Connect(
        sim_env.get_output_port_by_name("switching_time"),
        controller.get_input_port(2)
    )
    builder.Connect(
        sim_env.get_output_port_by_name("alip_state"),
        controller.get_input_port(3)
    )


    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_env.cassie_sim.SetPlantInitialConditionFromIK(
        diagram,
        context,
        np.zeros((3,)),
        0.15,
        1.0
    )
    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(5.0)


if __name__ == "__main__":
    main()