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

    2023.11.20 new setting (Wei-Cheng)
    4. desired velocity limited to reasonable region (a sector in the front)
    5. constant input replaced by LQR reference + Guassian noise, record the index by project it to the nearest grid
"""

import pdb
import os
import numpy as np
from copy import deepcopy
from tqdm import tqdm
from typing import Dict, Tuple, List
import argparse
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

from pydairlib.perceptive_locomotion.systems.alip_lqr_rl import (
    AlipFootstepLQROptions,
    AlipFootstepLQR,
    calc_collision_cost_grid
)

from pydairlib.perceptive_locomotion.systems. \
cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)

# Can use DrawAndSaveDiagramGraph for debugging if necessary
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"


def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepLQR, Diagram]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)    
    controller = sim_env.AddToBuilderWithFootstepController(builder, AlipFootstepLQR)
    ####
    observation = sim_env.AddToBuilderObservations(builder)
    reward = sim_env.AddToBuilderRewards(builder)
    builder.ExportInput(controller.get_input_port_by_name("action_ue"), "actions")
    ####
    diagram = builder.Build()
    #DrawAndSaveDiagramGraph(diagram, '../ALIPLQR_RL')
    return sim_env, controller, diagram


def run_experiment(sim_params: CassieFootstepControllerEnvironmentOptions,
                   num_data=10, job_id=None):
    sim_env, controller, diagram = build_diagram(sim_params)
    simulator = Simulator(diagram)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/initial_conditions_2.npz'
        )
    )

    # New: parametrize the desired velocity to be a sector (theta and |v|) forward
    v_des_theta = np.pi / 6
    v_des_norm = 1.0

    # initialize data list
    data = []

    # loop for num_data times to get the residual and x_desired in LQR stage
    job_id = 1 if job_id is None else job_id
    with tqdm(total=num_data, desc=f'Data collection Job {job_id}') as progress_bar:
        for i in range(num_data):
            datapoint = ic_generator.random()
            v_theta = np.random.uniform(-v_des_theta, v_des_theta)
            v_norm = np.random.uniform(0.2, v_des_norm)
            datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()

            data_i = get_data_sequence(
                sim_env, controller, diagram, simulator, datapoint
            )
            data += data_i
            progress_bar.update(1)
    return data


def initialize_sim(sim_env: CassieFootstepControllerEnvironment,
                   controller: AlipFootstepLQR,
                   diagram: Diagram,
                   datapoint: Dict) -> Dict:

    context = diagram.CreateDefaultContext()

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds

    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + datapoint['phase'] + t_ds
    context.SetTime(t_init)

    # set the context state with the initial conditions from the datapoint
    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )
    return {'root': context, 'sim': sim_context, 'controller': controller_context}


def get_noisy_footstep_command_indices(
        footstep_command: np.ndarray, hmap: np.ndarray,
        collision_cost: np.ndarray, variance: float) -> Tuple[int, int]:
    # New: the input should be LQR reference (ud) + noise
    wx = np.random.normal(loc=0, scale=np.sqrt(variance))
    wy = np.random.normal(loc=0, scale=np.sqrt(variance))

    # clip the noise to make outlier rejection easier
    wx = np.clip(wx, -3 * np.sqrt(variance), 3 * np.sqrt(variance))
    wy = np.clip(wy, -3 * np.sqrt(variance), 3 * np.sqrt(variance))

    noisy_footstep_command = footstep_command + np.array([wx, wy])
    distance = np.linalg.norm(
        hmap[:2] - np.expand_dims(
            np.expand_dims(noisy_footstep_command, 1), 1),
        axis=0
    )

    cost = collision_cost + distance

    i, j = np.unravel_index(cost.argmin(), cost.shape)
    return i, j


def get_data_sequence(sim_env: CassieFootstepControllerEnvironment,
                      controller: AlipFootstepLQR,
                      diagram: Diagram,
                      simulator: Simulator,
                      datapoint: Dict) -> List[Dict]:

    model_datapoint = deepcopy(datapoint)
    model_datapoint['stance'] = 0 if model_datapoint['stance'] == 'left' else 1

    def check_termination(diagram_context) -> bool:
        plant = sim_env.cassie_sim.get_plant()
        plant_context = plant.GetMyContextFromRoot(diagram_context)
        left_toe_pos = plant.CalcPointsPositions(
            plant_context, plant.GetBodyByName("toe_left").body_frame(),
            np.array([0.02115, 0.056, 0.]), plant.world_frame()
        )
        right_toe_pos = plant.CalcPointsPositions(
            plant_context, plant.GetBodyByName("toe_right").body_frame(),
            np.array([0.02115, 0.056, 0.]), plant.world_frame()
        )
        com = plant.CalcCenterOfMassPositionInWorld(plant_context)

        z1 = com[2] - left_toe_pos[2]
        z2 = com[2] - right_toe_pos[2]
        return z1 < 0.2 or z2 < 0.2

    def make_new_datapoint(diagram_context) -> Dict:
        plant = sim_env.cassie_sim.get_plant()
        plant_context = plant.GetMyContextFromRoot(diagram_context)
        return {
            'stance': model_datapoint['stance'],
            'phase': 0.0,
            'desired_velocity': model_datapoint['desired_velocity'],
            'q': plant.GetPositions(plant_context),
            'v': plant.GetVelocities(plant_context)
        }
    data = []
    max_steps = 50
    contexts = initialize_sim(
        sim_env, controller, diagram, datapoint
    )
    simulator.reset_context(contexts['root'])
    simulator.Initialize()

    # datapoint values set before we get it here:
    # ['stance', 'desired_velocity', 'phase', 'initial_swing_foot_pos', 'q', 'v']

    for i in range(max_steps):
        success = get_residual(sim_env, controller, simulator, contexts, datapoint)
        if not success or check_termination(contexts['root']):
            break
        data.append(datapoint)
        datapoint = make_new_datapoint(contexts['root'])

    return data


def get_residual(sim_env: CassieFootstepControllerEnvironment,
                 controller: AlipFootstepLQR,
                 simulator: Simulator,
                 contexts: Dict,
                 datapoint: Dict) -> bool:

    context = contexts['root']

    xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(
        contexts['controller']
    )
    xd = xd_ud[:4]
    ud = xd_ud[4:]
    u_fb = controller.get_output_port_by_name('footstep_command').Eval(
        contexts['controller']
    )[:2]

    hmap_center = np.array([ud[0], ud[1], 0])
    hmap = sim_env.get_heightmap(contexts['sim'], center=hmap_center)
    collision_cost = calc_collision_cost_grid(hmap[0], hmap[1], ud)

    i, j = get_noisy_footstep_command_indices(u_fb, hmap, collision_cost, .04 ** 2)

    # stabilizing footstep is too far from nominal, assume we are falling or fallen
    if np.linalg.norm(u_fb - hmap[:2, i, j]) > 0.5:
        return False

    sim_env.get_input_port_by_name("footstep_command").FixValue(
        context=contexts['sim'],
        value=hmap[:, i, j]
    )
    datapoint['x_k'] = controller.get_output_port_by_name('x').Eval(
        contexts['controller']
    ) - xd
    datapoint['V_kp1'] = controller.get_next_value_estimate_for_footstep(
        hmap[:, i, j], contexts['controller']
    )

    # Timing aliases
    t_init = context.get_time()
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 1e-2

    # Constant footstep command through the end of double stance
    simulator.AdvanceTo(t_init - datapoint['phase'] + t_s2s - t_eps)

    # -----------LQR input stage (DS + the end of next SS) ----------------#
    t = context.get_time()
    while t < (t_init - datapoint['phase'] + t_s2s) + (t_ss - 3 * t_eps):
        command = controller.get_output_port_by_name(
            'footstep_command'
        ).Eval(contexts['controller']).ravel()
        command[2] = sim_env.query_heightmap(contexts['sim'], command)
        sim_env.get_input_port_by_name("footstep_command").FixValue(
            context=contexts['sim'],
            value=command
        )
        simulator.AdvanceTo(t + 1e-2)
        t = context.get_time()

    xd = controller.get_output_port_by_name('lqr_reference').Eval(
        contexts['controller']
    )[:4]
    datapoint['x_kp1'] = controller.get_output_port_by_name('x').Eval(
        contexts['controller']
    ) - xd
    datapoint['V_k'] = controller.get_value_estimate(contexts['controller'])
    datapoint['residual'] = datapoint['V_k'] - datapoint['V_kp1']
    datapoint['hmap'] = hmap[-1, :, :]
    datapoint['U'] = hmap[:-1, :, :] - np.expand_dims(np.expand_dims(ud, 1), 1)
    datapoint['i'] = i
    datapoint['j'] = j
    datapoint['u_k'] = hmap[:2, i, j] - ud
    datapoint['footstep_command'] = hmap[:, i, j]

    # Potentially get the simulation ready for the next step if chaining sequences
    # TODO (@Brian-Acosta) Adjust timing to account for delay when using simulated perception stack
    beginning_of_next_phase = round(context.get_time() / t_s2s) * t_s2s + t_ds
    if context.get_time() < beginning_of_next_phase:
        simulator.AdvanceTo(beginning_of_next_phase + t_eps)

    return True


def data_process(i, q, visualize, terrain):
    num_data = 200
    sim_params = CassieFootstepControllerEnvironmentOptions(
        terrain=terrain, visualize=visualize
    )
    data = run_experiment(sim_params, num_data, i)
    q.put(data)


def main(save_file: str, visualize: bool):
    min_jobs = 100
    jobs_per_group = int(os.cpu_count() / 2)
    groups = int(np.ceil(min_jobs / jobs_per_group))

    job_queue = multiprocessing.Queue()
    job_list = []

    stairs = os.path.join(
        perception_learning_base_folder,
        'params/stair_curriculum.yaml'
    )

    wavy = os.path.join(
        perception_learning_base_folder,
        'params/wavy_terrain.yaml'
    )
    
    flat = os.path.join(
        perception_learning_base_folder,
        'params/flat.yaml'
    )

    testing = True
    if testing:
        data_process(0, job_queue, True, wavy)

    for group in range(groups):
        terrain = stairs if group % 4 == 0 else wavy
        for i in range(jobs_per_group):
            process = multiprocessing.Process(
                target=data_process, args=(i, job_queue, visualize, terrain)
            )
            job_list.append(process)
            process.start()
        results = [job_queue.get() for job in job_list]

        data_rearrange = []
        for i in range(len(results)):
            for res in results[i]:
                data_rearrange.append(res)
        group_save_file = f'{save_file}-{group}.npz'
        data_path = os.path.join(perception_learning_base_folder, 'tmp', group_save_file)
        np.savez(data_path, data_rearrange)
        print(f'Data saved to {data_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--save_file',
        type=str,
        default='data',
        help='Filepath where data should be saved'
    )
    parser.add_argument(
        '--visualize',
        type=bool,
        default=False,
        help='Whether to visualize the data collection procedure '
             '(WARNING - ONLY USES ONE THREAD)'
    )
    args = parser.parse_args()
    main(args.save_file, args.visualize)
