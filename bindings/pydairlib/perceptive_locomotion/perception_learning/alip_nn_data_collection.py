import pdb
import os
import numpy as np
import torch
from tqdm import tqdm
from typing import Dict, Tuple
import argparse
import multiprocessing
from pydrake.geometry import Rgba

# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.geometry import Meshcat
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
    #cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)

    footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)

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
    return z1 < 0.2 or z2 < 0.2

def run(sim_env, controller, diagram, simulate_perception=False, plot=False):
    
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic_new.npz'
        )
    )
    
    datapoint = ic_generator.random()
    #datapoint = ic_generator.choose(1)
    
    v_des_theta = 0.1
    v_des_norm = 0.8
    v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    v_norm = np.random.uniform(0.2, v_des_norm)
    datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()

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
    t_init = datapoint['stance'] * t_s2s + t_ds + t_eps + datapoint['phase']
    context.SetTime(t_init)

    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )

    ### Flip initial conditions ###
    # from pydrake.math import RigidTransform, RotationMatrix
    
    # simulator.reset_context(context)
    # x = np.concatenate((datapoint['q'], datapoint['v']))
    # print(x)
    
    # xnew = np.zeros((45))
    # plant = sim_env.cassie_sim.get_plant()
    # plant_context = plant.GetMyContextFromRoot(context)
    # R_WB = plant.EvalBodyPoseInWorld(context=plant_context,
    #         body=plant.GetBodyByName("pelvis")).rotation().matrix()
    # #Construct a matrix R, which reflects 3d vectors across the x-z plane of the pelvis
    # n = np.array([[0], [1], [0]])
    # R = np.eye(3) - 2 * n @ n.T
    # new_R_WB = R @ R_WB
    # new_R_WB[:, 1] *= -1
    # q = RotationMatrix(new_R_WB).ToQuaternion().wxyz()
    # # Assign the transformed state
    # xnew[0:4] = q
    # xnew[7:23] = x[7:23]
    # xnew[29:45] = x[29:45]
    # xnew[4:7] = R @ x[4:7]
    # xnew[26:29] = R @ x[26:29]
    # xnew[23:26] = R @ x[23:26]
    # print(xnew)

    # xnew[7:15], xnew[15:23] = xnew[15:23], xnew[7:15].copy()
    # xnew[7:9] = -xnew[7:9]
    # xnew[15:17] = -xnew[15:17]
    # xnew[29:37], xnew[37:45] = xnew[37:45], xnew[29:37].copy()
    # xnew[29:31] = -xnew[29:31]
    # xnew[37:39] = -xnew[37:39]

    # print(xnew)
    # input("=====")

    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(context)

    simulator.reset_context(context)
    ALIPtmp = []
    FOOTSTEPtmp = []
    HMAPtmp = []
    DMAPtmp = []
    VDEStmp = []
    JOINTtmp = []
    ACTtmp = []
    terminate = False
    time = 0.0

    from scipy.spatial.transform import Rotation as R

    for i in range(1, 401): # 20 seconds
        if check_termination(sim_env, context, time):
            terminate = True
            break
        simulator.AdvanceTo(t_init + 0.05*i)
        if simulate_perception:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            fsm = controller.get_input_port_by_name('fsm').Eval(controller_context)
            joint_angle = states[:23]
            actuator = states[-10:]
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)
            
            #sim_context = sim_env.GetMyMutableContextFromRoot(diagram_context)
            track_error = sim_env.get_output_port_by_name('swing_ft_tracking_error').Eval(sim_context)
            # yaw = sim_env.get_output_port_by_name('pelvis_yaw').Eval(sim_context)
            
            # fb_frame = plant.GetBodyByName("pelvis").body_frame()
            # fb_matrix = fb_frame.CalcPoseInWorld(plant_context).rotation().matrix()
            # fb_dir = np.dot(fb_matrix[:, 0] ,np.array([1, 0, 0]))
            # rotation = R.from_matrix(fb_matrix)
            # euler_angles = rotation.as_euler('xyz', degrees=False)
            # print(np.linalg.norm(euler_angles[2]))
            # print(fb_dir)
            
            # Depth map
            dmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            dmap = dmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Height map
            hmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map_query']
            ).get_value()

            hmap = hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Plot depth image
            if plot:
                grid_world = dmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                dmap_query.plot_surface(
                    "dmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))
                
                grid_world = hmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                hmap_query.plot_surface(
                    "hmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.95, 0.5, 0.5, 1.0))

        else:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            joint_angle = states[:23]
            actuator = states[45:55]
            
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

        HMAPtmp.append(hmap)
        if simulate_perception:
            DMAPtmp.append(dmap)
        ALIPtmp.append(alip)
        VDEStmp.append(datapoint['desired_velocity'])
        JOINTtmp.append(joint_angle)
        ACTtmp.append(actuator)
        FOOTSTEPtmp.append(footstep)
        
        time = context.get_time()-t_init

    return HMAPtmp, DMAPtmp, ALIPtmp, VDEStmp, JOINTtmp, ACTtmp, FOOTSTEPtmp, terminate, time


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/best_model_checkpoint.pth') # path of trained U-Net
    
    # True if using depth sensor
    sim_params.simulate_perception = True

    # Visualization is False by default
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    
    HMAP = []
    DMAP = []
    ALIP = []
    VDES = []
    JOINT = []
    ACT = []
    FOOTSTEP = []
    
    actions = []
    observations = []

    random_terrain = True

    input("Starting...")

    for i in range(100):
        if random_terrain:
            #rand = np.random.randint(1, 2)
            rand = 2

            if rand == 1:
                # Terrain without blocks
                rand = np.random.randint(1, 8)
                if rand in [1, 2, 3]:
                    terrain = 'params/stair_curriculum.yaml'
                elif rand in [4, 5, 6]:
                    terrain = 'params/wavy_terrain.yaml'
                else:
                    terrain = 'params/flat.yaml'
            
            else:
                #rand = np.random.randint(1, 3)
                rand = 2
                if rand == 1:
                    rand = np.random.randint(0, 1000)
                    terrain = f'params/flat/flat_{rand}.yaml'

                else:
                    rand = np.random.randint(0, 1000)
                    terrain = f'params/medium/stair_up/ustair_{rand}.yaml'
        else:
            terrain = 'params/stair_curriculum.yaml'
        #terrain = 'params/hard/stair_up/ustair_1.yaml'
        #terrain = 'params/new/flat/flat_11.yaml'
        terrain = 'params/flat_0.yaml'
        
        os.path.join(perception_learning_base_folder, terrain)
        sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
        # sim_params.terrain = 'terrain/stair_0.yaml'
        sim_env, controller, diagram = build_diagram(sim_params, checkpoint_path, sim_params.simulate_perception)
        hmap, dmap, alip, vdes, joint, actuator, footstep, terminate, time = run(sim_env, controller, diagram, sim_params.simulate_perception, plot=True)
        print(f"Iteration {i}: Terminated in {time} seconds in {terrain}.")

        if not terminate:
            HMAP.extend(hmap)
            DMAP.extend(dmap)
            ALIP.extend(alip)
            VDES.extend(vdes)
            JOINT.extend(joint)
            ACT.extend(actuator)
            FOOTSTEP.extend(footstep)

        del sim_env, controller, diagram

    print(f"Number of collected datapoints is: {np.array(ALIP).shape[0]}")
    
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/HMAP.npy', HMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/DMAP.npy', DMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/ALIP.npy', ALIP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/VDES.npy', VDES
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/JOINT.npy', JOINT
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/ACT.npy', ACT
    )

    print("Saving actions and observations...")

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/actions.npy', FOOTSTEP
    )

    DMAP = np.asarray(DMAP)
    ALIP = np.asarray(ALIP)
    VDES = np.asarray(VDES)
    JOINT = np.asarray(JOINT)
    ACT = np.asarray(ACT)

    DMAP = DMAP.reshape((DMAP.shape[0], -1))
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/observations.npy', np.concatenate((DMAP, ALIP, VDES, JOINT, ACT), axis=1)
    )

if __name__ == '__main__':
    main()
