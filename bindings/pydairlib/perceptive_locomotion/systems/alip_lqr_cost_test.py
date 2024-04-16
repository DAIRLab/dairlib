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
    ZeroOrderHold,
    VectorLogSink,
    LogVectorOutput,
)

from pydairlib.perceptive_locomotion.systems.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.perception_learning.true_cost_system import (
    CumulativeCost)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np
import os
from grid_map import GridMap
from pydairlib.perceptive_locomotion import vision_utils

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/terrain.yaml'
    #sim_params.simulate_perception = True
    sim_params.visualize = True
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    builder = DiagramBuilder()

    controller = AlipFootstepLQR(controller_params)
    footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)
    builder.AddSystem(footstep_zoh)
    builder.AddSystem(sim_env)

    #desired_velocity = ConstantVectorSource(np.array([0.4, 0]))
    builder.AddSystem(controller)
    #builder.AddSystem(desired_velocity)

    # controller give footstep command to sim_environment (i.e. cassie)
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )

    # external user assign desire velocity to controller
    #builder.Connect(
    #    desired_velocity.get_output_port(),
    #    controller.get_input_port_by_name("desired_velocity")
    #)

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
    ####
    cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)
    cost_zoh = ZeroOrderHold(0.05, 1) # only need to log the cost at sparse intervals, since it updates once per stride
    cost_logger = VectorLogSink(1)
    builder.AddSystem(cost_zoh)
    builder.AddSystem(cost_logger)
    builder.Connect(
        cost_system.get_output_port(),
        cost_zoh.get_input_port()
    )
    builder.Connect(
        cost_zoh.get_output_port(),
        cost_logger.get_input_port()
    )
    ####
    footstep_logger = VectorLogSink(3)
    state_logger = VectorLogSink(4)
    builder.AddSystem(footstep_logger)
    builder.AddSystem(state_logger)
    builder.Connect(
        controller.get_output_port_by_name('action'),
        footstep_logger.get_input_port()
    )
    builder.Connect(
        controller.get_output_port_by_name('x_xd'),
        state_logger.get_input_port()
    )

    diagram = builder.Build()
    #DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/initial_conditions_2.npz'
        )
    )
    datapoint = ic_generator.random()
    v_des_theta = np.pi / 6
    v_des_norm = 1.0
    v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    v_norm = np.random.uniform(0.2, v_des_norm)
    datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()
    #datapoint = ic_generator.choose(0)
    #datapoint['desired_velocity'] = np.array([ 1.2, 0. ])
    #datapoint['desired_velocity'] = np.array([ 0.8, -0.2 ])
    #print(datapoint['desired_velocity'])
    
    context = diagram.CreateDefaultContext()

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 0.01  # small number that prevent impact

    # grab the sim and controller contexts for convenience
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    cost_system_context = cost_logger.GetMyMutableContextFromRoot(context)
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

    simulator.reset_context(context)
    simulator.Initialize()

    #plant = sim_env.cassie_sim.get_plant()
    #plant_context = plant.GetMyContextFromRoot(context)
    #fb_frame = plant.GetBodyByName("pelvis").body_frame()
    #bf_vel = fb_frame.CalcSpatialVelocity(
    #    plant_context, plant.world_frame(), fb_frame).translational()

    #left_toe_pos = plant.CalcPointsPositions(
    #    plant_context, plant.GetBodyByName("toe_left").body_frame(),
    #    np.array([0.02115, 0.056, 0.]), plant.world_frame()
    #)
    #right_toe_pos = plant.CalcPointsPositions(
    #    plant_context, plant.GetBodyByName("toe_right").body_frame(),
    #    np.array([0.02115, 0.056, 0.]), plant.world_frame()
    #)
    #com = plant.CalcCenterOfMassPositionInWorld(plant_context)
    #z1 = com[2] - left_toe_pos[2]
    #z2 = com[2] - right_toe_pos[2]

    #simulator.set_target_realtime_rate(1.0)
    #simulator.AdvanceTo(t_init + 20)
    init = context.get_time()
    t_next = 0.05
    while t_next < 20:
        simulator.AdvanceTo(init+t_next)
        t_next += 0.05
    
    cost_log = cost_logger.FindLog(context).data()
    foot_log = footstep_logger.FindLog(context).data()
    state_log = state_logger.FindLog(context).data()
    #print(context.get_time())
    #np.save('foot5', foot_log)
    #np.save('state5', state_log)
    print(cost_logger.FindLog(context).data()[-1][-1])
    #print(foot_log.shape)
    #print(foot_log)
    #print(state_log.shape)
    #print(state_log)

if __name__ == "__main__":
    main()
