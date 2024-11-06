import pdb
import os
import numpy as np
import torch
from tqdm import tqdm
from typing import Dict, Tuple
import argparse
import multiprocessing
from pydrake.geometry import Rgba
import math
from pydrake.common import RandomGenerator
from pydrake.multibody.tree import BodyIndex, JointIndex, RevoluteJoint, PrismaticJoint

# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.geometry import Meshcat
from pydrake.math import RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
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
    ZeroOrderHold,
    VectorLogSink,
    LogVectorOutput,
)

from pydairlib.perceptive_locomotion.perception_learning.alip_nn_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepNNLQR,
    calc_collision_cost_grid
)
from pydairlib.perceptive_locomotion.perception_learning.utils.terrain_utils import (
    make_stairs, random_stairs
)

from pydairlib.perceptive_locomotion.perception_learning.inference.torch_utils \
    import get_device, tile_and_concatenate_inputs

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)
from pydairlib.perceptive_locomotion.perception_learning.utils.true_cost_system import (
    CumulativeCost)

# Can use DrawAndSaveDiagramGraph for debugging if necessary
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"


def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions,
                  checkpoint_path, simulate_perception=False) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepNNLQR, Diagram, LogVectorOutput]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller = sim_env.AddToBuilderWithFootstepController(
        builder, AlipFootstepNNLQR, model_path=checkpoint_path, elevation_map=simulate_perception
    )
    cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)

    footstep_zoh = ZeroOrderHold(1.0 / 40.0, 3)

    builder.AddSystem(footstep_zoh)
    if simulate_perception:
        builder.Connect(
            sim_env.get_output_port_by_name("height_map"),
            controller.get_input_port_by_name("height_map")
        )
        builder.Connect(
            sim_env.get_output_port_by_name("height_map_query"),
            controller.get_input_port_by_name("height_map_query")
        )
    else:    
        builder.Connect(
            sim_env.get_output_port_by_name('height_map'),
            controller.get_input_port_by_name('height_map')
        )

    builder.Connect(
        sim_env.get_output_port_by_name('state'),
        controller.get_input_port_by_name('xut')
    )

    builder.Connect(
        sim_env.get_output_port_by_name('gt_x_u_t'),
        controller.get_input_port_by_name('gt_x_u_t')
    )    
    
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )
    diagram = builder.Build()
    #DrawAndSaveDiagramGraph(diagram, '../AlipNNLQR_DataCollection')
    return sim_env, controller, diagram

def check_termination(sim_env, diagram_context, time) -> bool:
    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    sim_context = sim_env.GetMyMutableContextFromRoot(diagram_context)
    track_error = sim_env.get_output_port_by_name('swing_ft_tracking_error').Eval(sim_context)

    front_contact_pt = np.array((-0.0457, 0.112, 0))
    rear_contact_pt = np.array((0.088, 0, 0))

    toe_left_rotation = plant.GetBodyByName("toe_left").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
    left_toe_direction = toe_left_rotation @ (front_contact_pt - rear_contact_pt)
    left_angle = abs(np.arctan2(left_toe_direction[2], np.linalg.norm(left_toe_direction[:2])))
    
    toe_right_rotation = plant.GetBodyByName("toe_right").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
    right_toe_direction = toe_right_rotation @ (front_contact_pt - rear_contact_pt)
    right_angle = abs(np.arctan2(right_toe_direction[2], np.linalg.norm(right_toe_direction[:2])))

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

    scene_graph = sim_env.get_output_port_by_name('scene_graph').Eval(sim_context)
    front_contact_pt = np.array((-0.0457, 0.112, 0))
    rear_contact_pt = np.array((0.088, 0, 0))
    toe_axis = front_contact_pt - rear_contact_pt
    toe_axis /= np.linalg.norm(toe_axis)
    collision = 0.

    left_toe_p = plant.GetBodyByName("toe_left").EvalPoseInWorld(plant_context).translation() + (toe_left_rotation @ toe_axis) * 0.12
    left_distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=left_toe_p, threshold=1.0)
    for left_distances in left_distances:
        if left_distances.distance <= -0.011:
            return True

    right_toe_p = plant.GetBodyByName("toe_right").EvalPoseInWorld(plant_context).translation() + (toe_right_rotation @ toe_axis) * 0.12
    distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=right_toe_p, threshold=1.0)
    for right_distance in distances:
        if right_distance.distance <= -0.011:
            return True
    return z1 < 0.2 or z2 < 0.2 #or (track_error > 0.4 and time > 2.)

def run(sim_env, controller, diagram, terrain, simulate_perception=False, plot=False):
    
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic.npz'
        )
    )
    
    datapoint = ic_generator.random()
    v_x = 0.8
    v_y = 0.2
    if terrain == 'no_obs':
        v_y = 0.4
        vx = np.random.uniform(-v_x, v_x)
        vy = np.random.uniform(-v_y, v_y)
    else:
        vx = np.random.uniform(0.0, v_x)
        vy = np.random.uniform(-v_y, v_y)
    datapoint['desired_velocity'] = np.array([vx, vy]).flatten()
    # print(datapoint['desired_velocity'])

    simulator = Simulator(diagram)
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
    t_init = datapoint['stance'] * t_s2s + t_eps + t_ds + datapoint['phase']
    context.SetTime(t_init)
    
    if terrain == 'stair':
        rand = np.random.randint(1,4)
        if rand in [1,2]:
            yaw = 0.0 # Upstair
            pos = np.random.uniform(low=-.1, high=.5)
        else:
            yaw = math.pi # Downstair
            pos = np.random.uniform(low=-.5, high=.1)
        
        rand = np.random.randint(-6,7)
        pos = np.random.uniform(low=-.5, high=.5)
        datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
    else:
        yaw = np.random.uniform(low=-math.pi, high=math.pi)
    
    quat = datapoint['q'][:4]
    quat = quat / np.linalg.norm(quat)
    R_WP = RotationMatrix(Quaternion(quat))

    wRp = RotationMatrix.MakeZRotation(yaw) @ R_WP
    q_pelvis = wRp.ToQuaternion().wxyz()
    w_pelvis = wRp @ datapoint['v'][:3]
    v_pelvis = wRp @ datapoint['v'][3:6]
    datapoint['q'][:4] = q_pelvis
    datapoint['v'][:3] = w_pelvis
    datapoint['v'][3:6] = v_pelvis

    diagram.SetRandomContext(context, RandomGenerator())

    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )
    
    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(context)

    for body_index in range(23):
        body = plant.get_body(BodyIndex(body_index+1))
        mass = body.get_mass(plant.CreateDefaultContext())
        rand = np.random.uniform(low=.7, high=1.3)
        body.SetMass(plant_context, mass * rand)

    for joint_index in range(plant.num_joints()):
        joint = plant.get_joint(JointIndex(joint_index))
        if isinstance(joint, (RevoluteJoint, PrismaticJoint)):
            damping_value = joint.default_damping()
            random_damping_factor = np.random.uniform(0.5, 2.5)
            new_damping = random_damping_factor * damping_value
            joint.SetDamping(plant_context, new_damping)

    Q = controller.params.Q
    R = controller.params.R

    simulator.reset_context(context)
    ALIPtmp = []
    FOOTSTEPtmp = []
    DMAPtmp = []
    VDEStmp = []
    JOINTtmp = []
    terminate = False
    time = 0.0
    for i in range(1, 401): # 10 seconds

        if check_termination(sim_env, context, time):
            terminate = True
            break
        simulator.AdvanceTo(t_init + 0.025*i)

        if simulate_perception:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            gt_states = controller.get_input_port_by_name('gt_x_u_t').Eval(controller_context)
            joint_angle = states[7:23]
            gt_joint_angle = gt_states[:23]

            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)
            u = footstep[:2]

            # Depth map
            dmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            dmap = dmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Height map
            # hmap_query = controller.EvalAbstractInput(
            #     controller_context, controller.input_port_indices['height_map_query']
            # ).get_value()

            # hmap = hmap_query.calc_height_map_stance_frame(
            #     np.array([ud[0], ud[1], 0])
            # )

            # Plot depth image
            if plot:
                grid_world = dmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                dmap_query.plot_surface(
                    "hmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        else:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x').Eval(controller_context)
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            joint_angle = states[7:23]
            
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)

            # get heightmap query object
            hmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            # query for cropped height map at nominal footstep location
            hmap = hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )
            if plot:
                grid_world = hmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                hmap_query.plot_surface(
                    "hmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        if simulate_perception:
            DMAPtmp.append(dmap.astype(np.float32))
        ALIPtmp.append(alip.astype(np.float32))
        VDEStmp.append(datapoint['desired_velocity'].astype(np.float32))
        JOINTtmp.append(joint_angle.astype(np.float32))
        FOOTSTEPtmp.append(footstep.astype(np.float32))

        time = context.get_time()-t_init

    return DMAPtmp, ALIPtmp, VDEStmp, JOINTtmp, FOOTSTEPtmp, terminate, time


def simulation_worker(sim_id, sim_params, checkpoint_path, perception_learning_base_folder):
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    random_terrain = True
    DMAP = []
    ALIP = []
    VDES = []
    JOINT = []
    RETURN = []
    FOOTSTEP = []
    time = 0.0
    print(f"Starting simulation {sim_id}...")
    np.random.seed(sim_id + 32 * 111) # + 32 to change seed value
    for i in range(10):
        if random_terrain:
            rand = np.random.randint(1, 11)
            rand = 10
            if rand in [1,2,3]:
                terrain_yaml = 'params/flat.yaml'
                terrain = 'no_obs'
            elif rand in [4,5]:
                rand = np.random.randint(0, 1000)
                terrain_yaml = f'params/slope/stair_{rand}.yaml'
                terrain = 'stair'
            else:
                rand = np.random.randint(0, 1000)
                terrain_yaml = f'params/stair/dustair_{rand}.yaml'
                terrain = 'stair' 

        else:
            terrain_yaml = 'params/stair_curriculum.yaml'

        sim_params.terrain = os.path.join(perception_learning_base_folder, terrain_yaml)
        sim_env, controller, diagram = build_diagram(sim_params, checkpoint_path, sim_params.simulate_perception)
        dmap, alip, vdes, joint, footstep, terminate, time = run(sim_env, controller, diagram, terrain, sim_params.simulate_perception, plot=False)
        print(f"Simulation {sim_id}, Iteration {i}: Terminated in {time} seconds in {terrain_yaml}.")

        if not terminate:
            DMAP.extend(dmap)
            ALIP.extend(alip)
            VDES.extend(vdes)
            JOINT.extend(joint)
            FOOTSTEP.extend(footstep)

        del sim_env, controller, diagram

    return DMAP, ALIP, VDES, JOINT, FOOTSTEP


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')  # path of trained U-Net
    
    sim_params.simulate_perception = True

    print("Starting multiprocessing simulations...")

    num_processes = 1
    pool = multiprocessing.Pool(processes=num_processes)

    tasks = [(i, sim_params, checkpoint_path, perception_learning_base_folder) for i in range(num_processes)]
    results = pool.starmap(simulation_worker, tasks)

    DMAP = []
    ALIP = []
    VDES = []
    JOINT = []
    FOOTSTEP = []

    for result in results:
        DMAP.extend(result[0])
        ALIP.extend(result[1])
        VDES.extend(result[2])
        JOINT.extend(result[3])
        FOOTSTEP.extend(result[4])

    print(f"Number of collected datapoints is: {np.array(ALIP).shape[0]}")
    
    # np.save(
    #     f'{perception_learning_base_folder}/tmp/data_collection'
    #     f'/DMAP.npy', DMAP.astype(np.float32)
    # )
    # np.save(
    #     f'{perception_learning_base_folder}/tmp/data_collection'
    #     f'/ALIP.npy', ALIP.astype(np.float32)
    # )
    # np.save(
    #     f'{perception_learning_base_folder}/tmp/data_collection'
    #     f'/VDES.npy', VDES.astype(np.float32)
    # )
    # np.save(
    #     f'{perception_learning_base_folder}/tmp/data_collection'
    #     f'/JOINT.npy', JOINT.astype(np.float32)
    # )

    print("Saving actions and observations ...")

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/actions.npy', FOOTSTEP
    )

    DMAP = np.asarray(DMAP, dtype=np.float32)
    ALIP = np.asarray(ALIP, dtype=np.float32)
    VDES = np.asarray(VDES, dtype=np.float32)
    JOINT = np.asarray(JOINT, dtype=np.float32)
    DMAP = DMAP.reshape((DMAP.shape[0], -1))

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/observations.npy', np.concatenate((DMAP, ALIP, VDES, JOINT), axis=1)
    )

if __name__ == '__main__':
    main()
