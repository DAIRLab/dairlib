import os

import numpy as np

from pydairlib.perceptive_locomotion.bench.bench_harness import (
    BenchEnvOptions,
    BenchHarness
)

from pydairlib.perceptive_locomotion.diagrams import (
    AlipMPFCDiagram
)

from pydairlib.systems import DrawAndSaveDiagramGraph

import pydairlib.perceptive_locomotion.terrain_segmentation as terrain_seg

from pydrake.systems.all import (
    State,
    Diagram,
    EventStatus,
    Context,
    Simulator,
    SimulatorStatus,
    InputPort,
    OutputPort,
    LeafSystem,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    ZeroOrderHold,
)

from pydrake.geometry import Meshcat
from pydairlib.multibody import SquareSteppingStoneList, ReExpressWorldVector3InBodyYawFrame

def add_mpc_perception_deps(builder, env, mpc):
    segmentation = builder.AddSystem(
        terrain_seg.TerrainSegmentationSystem({
                'curvature_criterion': terrain_seg.curvature_criterion,
                'variance_criterion': terrain_seg.variance_criterion,
            },
        update_period=1.0/30.0)
    )
    decomposition = builder.AddSystem(
        terrain_seg.ConvexTerrainDecompositionSystem()
    )
    builder.Connect(
        env.get_output_port_by_name('elevation_map'),
        segmentation.get_input_port(),
    )
    builder.Connect(
        segmentation.get_output_port(),
        decomposition.get_input_port()
    )
    builder.Connect(
        decomposition.get_output_port(),
        mpc.get_input_port_footholds()
    )
    builder.Connect(
        env.get_output_port_by_name('lcmt_robot_output'),
        mpc.get_input_port_state()
    )

def run(env, controller, diagram):
    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_context = env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)

    xvdes = np.random.uniform(-0.8, 0.8)
    yvdes = 0.#np.random.uniform(-0.4, 0.4)
    vdes_source = np.array([xvdes, yvdes]).flatten()
    print(vdes_source)
    controller.get_input_port_vdes().FixValue(
        context = controller_context,
        value = vdes_source
    )

    q = np.array([1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
        -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
        -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785])
    v = np.zeros((22,))
    t_init = 0.0
    context.SetTime(t_init)
    env.initialize_state(context, diagram, q, v)
    


def main():
    opts = BenchEnvOptions()
    opts.visualize = False
    opts.meshcat = Meshcat()

    env = BenchHarness(opts)
    controller = AlipMPFCDiagram(env.controller_plant, opts.mpfc_gains_yaml, 30)

    builder = DiagramBuilder()
    builder.AddSystem(env)
    builder.AddSystem(controller)

    add_mpc_perception_deps(builder, env, controller)

    builder.Connect(
        controller.get_output_port_mpc_output(),
        env.get_input_port_by_name("command")
    )

    #xvdes = np.random.uniform(0, 0.8)
    #vdes_source = builder.AddSystem(ConstantVectorSource(np.array([xvdes, 0.0])))

    # builder.Connect(
    #     vdes_source.get_output_port(),
    #     controller.get_input_port_vdes()
    # )

    diagram = builder.Build()

    # DrawAndSaveDiagramGraph(diagram, 'bench')
    # simulator = Simulator(diagram)
    # context = diagram.CreateDefaultContext()
    # controller_context = controller.GetMyMutableContextFromRoot(context)
    # np.random.seed()
    MSE = []
    DES = []
    TRUE = []
    simulator = Simulator(diagram)
    print("Friction: 1.1 slopy stair")
    for i in range(200):
        # simulator = Simulator(diagram)
        context = diagram.CreateDefaultContext()
        controller_context = controller.GetMyMutableContextFromRoot(context)
        sim_context = env.GetMyMutableContextFromRoot(context)

        # xvdes = np.random.uniform(0.2, 0.8)
        # vdes_source = np.array([xvdes, 0.]).flatten()
        xvdes = np.random.uniform(0., 0.4)
        #yvdes = np.random.uniform(-0.2, 0.2)
        yvdes = 0.
        vdes_source = np.array([xvdes, yvdes]).flatten()
        print(vdes_source)
        controller.get_input_port_vdes().FixValue(
            context = controller_context,
            value = vdes_source
        )

        # repeat the simulation a few times
        q = np.array([1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
        -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
        -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785])
        v = np.zeros((22,))

        env.initialize_state(context, diagram, q, v)
        simulator.reset_context(context)
        simulator.Initialize()
        # input('waiting')
        simulator.set_target_realtime_rate(3.0)
        t_init = 0.0
        context.SetTime(t_init)
        terminate = False

        def monitor(context):
            # time_limit = 10

            plant = env.cassie_sim.get_plant()
            plant_context = plant.GetMyContextFromRoot(context)
            # plant = sim_env.cassie_sim.get_plant()
            # plant_context = plant.GetMyContextFromRoot(diagram_context)

            sim_context = env.GetMyMutableContextFromRoot(context)
            track_error = env.get_output_port_by_name('swing_ft_tracking_error').Eval(sim_context)
            
            # if center of mas is 20cm 
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
            
            front_contact_pt = np.array((-0.0457, 0.112, 0))
            rear_contact_pt = np.array((0.088, 0, 0))

            toe_left_rotation = plant.GetBodyByName("toe_left").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
            left_toe_direction = toe_left_rotation @ (front_contact_pt - rear_contact_pt)
            left_angle = abs(np.arctan2(left_toe_direction[2], np.linalg.norm(left_toe_direction[:2])))
            
            toe_right_rotation = plant.GetBodyByName("toe_right").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
            right_toe_direction = toe_right_rotation @ (front_contact_pt - rear_contact_pt)
            right_angle = abs(np.arctan2(right_toe_direction[2], np.linalg.norm(right_toe_direction[:2])))

            # if context.get_time() > time_limit:
            #     return EventStatus.ReachedTermination(diagram, "Max Time Limit")
            
            if z1 < 0.2:
                print("Left Toe Exceeded")
                return EventStatus.ReachedTermination(diagram, "Left Toe Exceeded")

            if z2 < 0.2:
                print("Right Toe Exceeded")
                return EventStatus.ReachedTermination(diagram, "Right Toe Exceeded")

            scene_graph = env.get_output_port_by_name('scene_graph').Eval(sim_context)
            front_contact_pt = np.array((-0.0457, 0.112, 0))
            rear_contact_pt = np.array((0.088, 0, 0))
            toe_axis = front_contact_pt - rear_contact_pt
            toe_axis /= np.linalg.norm(toe_axis)
            collision = 0.
            left_toe_p = plant.GetBodyByName("toe_left").EvalPoseInWorld(plant_context).translation() + (toe_left_rotation @ toe_axis) * 0.12
            left_distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=left_toe_p, threshold=1.0)
            for distances in left_distances:
                if distances.distance <= -0.012:
                    print("Left Collision")
                    return EventStatus.ReachedTermination(diagram, "Left Collision")

            right_toe_p = plant.GetBodyByName("toe_right").EvalPoseInWorld(plant_context).translation() + (toe_right_rotation @ toe_axis) * 0.12
            distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=right_toe_p, threshold=1.0)
            for signed_distance in distances:
                if signed_distance.distance <= -0.012:
                    print("Right Collision")
                    return EventStatus.ReachedTermination(diagram, "Right Collision")

            # if track_error > 0.6 and (context.get_time() > 1.):
            #     print("Track Error")
            #     return EventStatus.ReachedTermination(diagram, "Track Error Exceeded")

            return EventStatus.Succeeded()

        simulator.set_monitor(monitor)

        for i in range(500):
            t_init += 0.025
            # if check_termination(env, context):
            #     terminate = True
            #     print('terminate')
            #     break
            pos_vel = env.get_output_port_by_name('state').Eval(sim_context)[:45]
            # print(pos_vel)
            plant = env.controller_plant
            plant_context = plant.CreateDefaultContext()
            plant.SetPositionsAndVelocities(plant_context, pos_vel)
            
            # Body Frame Velocity
            bf_frame = plant.GetBodyByName("pelvis").body_frame()
            bf_velocity = bf_frame.CalcSpatialVelocityInWorld(plant_context)
            bf_vel = bf_velocity.translational()
            bf_vel = ReExpressWorldVector3InBodyYawFrame(plant, plant_context, "pelvis", bf_vel)
            status = simulator.AdvanceTo(t_init)
            # print(status)
            # print(bf_vel[0])
            # print(np.linalg.norm(xvdes - bf_vel[0]))
            DES.append(xvdes)
            TRUE.append(bf_vel[0])
            if (status.reason() == SimulatorStatus.ReturnReason.kReachedTerminationCondition):
                # print(status.reason())
                # print(context.get_time())
                break
    
    DES = np.asarray(DES)
    TRUE = np.asarray(TRUE)
    
    mse = np.mean((DES - TRUE)**2)
    MSE.append(mse)
    
    MSE = np.asarray(MSE)
    print(np.mean(MSE))
    # np.save('MPC_flat.npy', MSE)
    # np.save('MPC_flat.npy', DES)
    # np.save('MPC_flat.npy', TRUE)

if __name__ == '__main__':
    main()
