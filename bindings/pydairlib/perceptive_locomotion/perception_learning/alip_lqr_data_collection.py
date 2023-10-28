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
    4. the corresponding hmap

    Many of the contents are grabbed from alip_lqr_cost_experiment,
    just iterate through different initial
    conditions and record them
    TODO::
    1. try to polish the functions, check and debug
    2. cut down unnecessary data or operations
"""

import os
import numpy as np
from typing import Dict, Tuple
import multiprocessing

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
    make_stairs, random_stairs
)

from pydairlib.perceptive_locomotion.perception_learning. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    perception_learning_base_folder,
    InitialConditionsServer
)

# Can use DrawAndSaveDiagramGraph for debugging if necessary
# from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph


def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepLQR, Diagram]:
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    controller = AlipFootstepLQR(controller_params)
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
    return sim_env, controller, diagram


def run_experiment(sim_params: CassieFootstepControllerEnvironmentOptions, num_data = 10):
    sim_env, controller, diagram = build_diagram(sim_params)
    simulator = Simulator(diagram)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/initial_conditions_2.npz'
        )
    )
    v_des_distr = np.linspace(-0.05, 0.05, 50)
    data = []

    # loop for num_data times to get the residual and x_desired in LQR stage
    for i in range(num_data):
        datapoint = ic_generator.random()
        datapoint['desired_velocity'] = np.random.choice(v_des_distr, size=2)
        get_residual(sim_env, controller, diagram, simulator, datapoint)
        data.append(datapoint)

    return data


def get_residual(sim_env: CassieFootstepControllerEnvironment,
                 controller: AlipFootstepLQR,
                 diagram: Diagram,
                 simulator: Simulator,
                 datapoint: Dict) -> None:
    # Make a new context per datapoint
    context = diagram.CreateDefaultContext()

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 0.01  # small number that prevent impact

    # grab the sim and controller contexts for convenience
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + t_ds + t_eps + datapoint['phase']
    context.SetTime(t_init)

    # set the context state with the initial conditions from the datapoint
    sim_env.cassie_sim.SetPlantInitialCondition(
        diagram,
        context,
        datapoint['q'],
        datapoint['v']
    )
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )
    ud = controller.get_output_port_by_name('lqr_reference').Eval(
        controller_context
    )[-2:]
    heightmap_center = np.zeros((3,))
    heightmap_center[:2] = ud
    datapoint['hmap'] = sim_env.get_heightmap(sim_context, center=heightmap_center)
    heightmap_center[-1] = sim_env.query_heightmap(sim_context, ud)
    datapoint['footstep_command'] = heightmap_center

    sim_env.get_input_port_by_name("footstep_command").FixValue(
        context=sim_context,
        value=datapoint['footstep_command']
    )

    simulator.reset_context(context)
    simulator.Initialize()

    controller_context = controller.GetMyMutableContextFromRoot(context)
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    datapoint['x_k'] = sim_env.get_output_port_by_name("alip_state").Eval(
        sim_context
    ).ravel()

    datapoint['V_kp1'] = controller.get_next_value_estimate_for_footstep(
        datapoint['footstep_command'], controller_context
    )

    # ----------- Constant footstep command stage ------------------------#
    simulator.AdvanceTo(t_init - datapoint['phase'] + t_ss)

    # -----------LQR input stage (DS + the end of next SS) ----------------#
    t = context.get_time()
    while t < (t_init - datapoint['phase'] + t_ss) + (t_s2s - 3 * t_eps):
        command = controller.get_output_port_by_name(
            'footstep_command'
        ).Eval(controller_context).ravel()
        command[2] = sim_env.query_heightmap(sim_context, command)
        sim_env.get_input_port_by_name("footstep_command").FixValue(
            context=sim_context,
            value=command
        )
        simulator.AdvanceTo(t + 1e-2)
        t = context.get_time()

    datapoint['V_k'] = controller.get_value_estimate(controller_context)
    datapoint['residual'] = datapoint['V_k'] - datapoint['V_kp1']

def data_process(i, q):
    num_data = 1000
    print("data_process", str(i))
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = random_stairs(0.3, 1, 0.2)
    data = run_experiment(sim_params, num_data)
    q.put(data)

def main():
    num_jobs = 16
    job_queue = multiprocessing.Queue()
    job_list = []

    for i in range(num_jobs):
        process = multiprocessing.Process(target = data_process,args = (i, job_queue))
        job_list.append(process)
        process.start()
    results = [job_queue.get() for job in job_list]

    data_rearrange = []
    for i in range(len(results)):
        for res in results[i]:
            data_rearrange.append(res)

    import pdb
    pdb.set_trace()


if __name__ == '__main__':
    main()
