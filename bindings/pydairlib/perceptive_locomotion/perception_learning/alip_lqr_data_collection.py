"""
    This file is used to test the data collection pipeline for the cassie LQR
    control
    Target is to create different initial condition for walking with the LQR
    controller
    
    generation method: let Cassie step with fixed constant control,
    and sample from the swing
    foot stage, all the samples can be regarded as a initial condition for
    LQR control
    
    Should vary the following things:
    1. different starting configurations of Cassie (q, v)
    2. left and right foot (stance, use 0,1 to represent)
    3. different timing during the swing foot stage (phase)
    4. desired velocity that is input to controller by user
    5. different hmap (constant input for swing foot stage)

    Should at least collect the following things:
    1. ALIP state
    2. (constant) input
    3. target end state for the LQR controller

    Many of the contents are grabbed from alip_lqr_cost_experiment,
    just iterate through different initial
    conditions and record them
    TODO::
    1. try to polish the functions, check and debug
    2. cut down unnecessary data or operations
"""

import os
import numpy as np
from typing import Tuple, Any

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
    ConstantVectorSource
)

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)
from pydairlib.perceptive_locomotion.perception_learning.terrain_utils import (
    make_stairs
)

from pydairlib.perceptive_locomotion.perception_learning. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    perception_learning_base_folder,
    InitialConditionsServer
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph


def run_experiment():
    ############################# block definition #############################
    # since we also test different desired speed, so need not to use constant
    # vector port for desired speed
    # instead just use port.FixValue to achieve the same effect
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = make_stairs(5.0, 1.0, 0.1, 11, 'up')
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    controller = AlipFootstepLQR(controller_params)

    ############################## building diagram ############################
    builder = DiagramBuilder()
    builder.AddSystem(sim_env)

    builder.AddSystem(controller)
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

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()

    ################### different initial conditions ###########################
    ss_duration = controller_params.single_stance_duration
    ds_duration = controller_params.double_stance_duration

    # define the distribution of the initial conditions, can be changed in
    # the future
    eps = 0.01  # small number that prevent impact
    Init_Generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder, 'tmp/initial_conditions_2.npz'
        )
    )  # call random() to get a dictionary with elements stance, phase, q ,v
    desired_velocity_dis = np.linspace(-0.05, 0.05, 50)
    Ts2s = ss_duration + ds_duration

    ################### data points collection #################################
    def residual_datapoint(initial_condition: list) -> Tuple[Any, Any, Any]:

        # -------------------- initial condition ------------------------------#
        q = initial_condition[0]
        v = initial_condition[1]
        phase = initial_condition[2]
        stance = initial_condition[3]
        footstep_command = initial_condition[4]
        desired_velocity = initial_condition[5]

        # --------constant input swing stage (to the end of SS) ---------------#
        t_init = ds_duration + eps + stance * (Ts2s) + phase
        context = diagram.CreateDefaultContext()
        context.SetTime(t_init)
        sim_env.cassie_sim.SetPlantInitialCondition(diagram, context, q, v)
        simulator.reset_context(context)
        # simulator.set_target_realtime_rate
        simulator.Initialize()

        controller_context = controller.GetMyMutableContextFromRoot(context)
        sim_context = sim_env.GetMyMutableContextFromRoot(context)
        alip_state = sim_env.get_output_port_by_name("alip_state").Eval(
            sim_context
        ).ravel()

        controller.get_input_port_by_name("desired_velocity").FixValue(
            context=controller_context,
            value=desired_velocity
        )
        V_kp1 = controller.get_next_value_estimate_for_footstep(
            footstep_command, controller_context
        )
        sim_env.get_input_port_by_name("footstep_command").FixValue(
            context=sim_context,
            value=footstep_command
        )
        simulator.AdvanceTo(t_init - phase + ss_duration)

        # -----------LQR input stage (DS + the end of next SS) ----------------#
        while context.get_time() < (t_init - phase + ss_duration) + (
                Ts2s - 3 * eps):
            command = controller.get_output_port_by_name(
                'footstep_command'
            ).Eval(controller_context).ravel()
            command[2] = sim_env.query_heightmap(sim_context, command)
            sim_env.get_input_port_by_name("footstep_command").FixValue(
                context=sim_context,
                value=command
            )
            simulator.AdvanceTo(context.get_time() + 1e-2)
        V_k = controller.get_value_estimate(controller_context)
        xd_ud = controller.get_output_port_by_name('footstep_command').Eval(
            controller_context
        ).ravel()
        x_desired = xd_ud[:4]

        return V_k - V_kp1, alip_state, x_desired

    # ---------------------- random initial condition setting -----------------#
    # number of data collected
    num_data = 1000  # 1000000

    # list to store the data and corresponding initial condition (probably
    # not needed?)
    alip_state_list = []
    input_list = []
    value_residual_list = []
    lqr_desired_state_list = []

    # loop for num_data times to get the residual and x_desired in LQR stage
    for i in range(num_data):
        # randomly generate the initial condition
        data = Init_Generator.random()
        q = data['q']
        v = data['v']

        # translate stance command into binary for the sack of computing
        # timespot
        stance_string = data['stance']
        stance = 0 if stance_string == 'left' else 1
        phase = data['phase']
        desired_velocity = np.random.choice(desired_velocity_dis, size=2)

        #  NOTE: set context align with the current stance, so that hmap is
        #  aligned with stance
        t_init = ds_duration + eps + stance * (Ts2s) + phase
        context.SetTime(t_init)
        sim_context = sim_env.GetMyMutableContextFromRoot(context)
        controller_context = controller.GetMyMutableContextFromRoot(context)
        controller.get_input_port_by_name("desired_velocity").FixValue(
            context=controller_context,
            value=desired_velocity
        )
        ud = controller.get_output_port_by_name('lqr_reference').Eval(
            controller_context
        )[-2:]
        heightmap_center = np.zeros((3,))
        heightmap_center[:2] = ud
        hmap = sim_env.get_heightmap(sim_context, center=heightmap_center)
        #  NOTE ENDS

        rng = np.random.default_rng()
        hmap_index1 = rng.integers(hmap.shape[1] / 3, 2 * hmap.shape[1] / 2)
        hmap_index2 = rng.integers(hmap.shape[2] / 3, 2 * hmap.shape[2] / 2)
        hmap_random = hmap[:, hmap_index1, hmap_index2]

        # run the simulation experiment to get residual and desired state for
        # LQR stage
        random_initial = [q, v, phase, stance, hmap_random, desired_velocity]
        value_residual, alip_state, lqr_desired_state = residual_datapoint(
            initial_condition=random_initial
        )

        # data list
        alip_state_list.append(alip_state)
        input_list.append(hmap_random)
        value_residual_list.append(value_residual)
        lqr_desired_state_list.append(lqr_desired_state)

    return value_residual_list, alip_state_list, input_list, \
        lqr_desired_state_list


if __name__ == '__main__':
    q_list, v_list, phase_list, stance_list, hmap_list, desired_velocity_list, \
        value_residual_list, lqr_desired_state_list = run_experiment()
