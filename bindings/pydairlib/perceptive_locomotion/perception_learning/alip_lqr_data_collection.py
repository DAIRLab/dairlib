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
    LogVectorOutput
)

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.perception_learning. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np
import io


''' 
    This file is used to test the data collection pipeline for the cassie LQR control
    Target is to create different initial condition for walking with the LQR controller
    
    generation method: let Cassie step with fixed constant control, and sample from the swing
    foot stage, all the samples can be regarded as a initial condition for LQR control
    
    Should vary the following things:
    1. left and right foot (start time be 0.1 or 0.52, because double stance is 0.1s and single
        stance is 0.32 s, so the two swing phase is 0.1-0.42 s and 0.52-0.84 s)
    2. different starting point of the swing foot, including changing parameters in 
        SetPlantInitialConditionFromIK (pelvis_vel, foot_spread, height)
    TODO :: SetPlantInitialConditionFromIK set things symmetrically, might need to use SetPlantInitialCondition
    
    Should collect the following things:
    ALIP state, input, and target end state for the LQR controller (i.e. 0.84s or 1.26s)
'''
def main():
    ########################################## creating blocks #########################################################
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )

    controller = AlipFootstepLQR(controller_params)
    desired_velocity = ConstantVectorSource(np.array([0.1, 0]))

    ############################################# build diagram ########################################################
    builder = DiagramBuilder()
    builder.AddSystem(sim_env)

    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)

    # controller give footstep command to sim_environment (i.e. cassie)
    builder.Connect(
        controller.get_output_port_by_name("footstep_command"),
        sim_env.get_input_port_by_name("footstep_command")
    )
    # external user assign desire velocity to controller
    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port_by_name("desired_velocity")
    )
    # sim_env (cassie) returns state_feedback to controller
    builder.Connect(
        sim_env.get_output_port_by_name("fsm"),
        controller.get_input_port_by_name("fsm")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("time_until_switch"),
        controller.get_input_port_by_name("time_until_switch")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("alip_state"),
        controller.get_input_port_by_name("state")
    )


    # also add data logger
    logger_ALIP_State = LogVectorOutput(sim_env.get_output_port_by_name("alip_state"),builder)
    logger_control = LogVectorOutput(controller.get_output_port_by_name("footstep_command"),builder)
    logger_desired_vel = LogVectorOutput(desired_velocity.get_output_port(),builder)

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_lqr.pdf')

    ########################################### simulation setting #####################################################
    # left and right foot setting
    ss_time = controller_params.single_stance_duration
    ds_time = controller_params.double_stance_duration

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()

    # height can vary from 0.88 to 1.28
    # footspread can vary from 0.35 to 0.15
    # pelvis velocity vary from
    # use for loop to change different initial conditions
    sim_env.cassie_sim.SetPlantInitialConditionFromIK(
        diagram,
        context,
        np.zeros((3,)),
        0.15,
        1.28
    )
    simulator.reset_context(context)
    context.SetTime(ds_time)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    # simulator.AdvanceTo(np.inf)
    simulator.AdvanceTo(5.0)

    state_log = logger_ALIP_State.FindLog(context).data() # (4, num_timstamp)
    control_log = logger_control.FindLog(context).data()  # (3, num_timstamp)
    desire_vel_log = logger_desired_vel.FindLog(context).data() # (2, num_timstamp)


if __name__ == "__main__":
    main()
