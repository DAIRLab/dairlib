import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ContactVisualizer,
    DiagramBuilder,
    LeafSystem,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    Simulator,
    SpatialForce,
    StartMeshcat,
    RigidTransform,
    Value,
    InitializeAutoDiff,
    ModelInstanceIndex,
    PiecewisePolynomial,
    VectorLogSink,
    EventStatus,
    CoulombFriction,
    RoleAssign
)

from c3 import *

from pydairlib.common import plot_styler
import matplotlib.pyplot as plt

from planar_box_controller import Controller

def main():
    plot_styler.PlotStyler.set_compact_styling()
    # only modifying mu of active block as we can compute the effective mu anyway
    save = False

    method = "robust"
    # method = "default"
    results_folder = '/home/yangwill/Documents/research/papers/dual_friction_margin/results/'
    np.set_printoptions(3, threshold=8, suppress=True)
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
    parser = Parser(plant)
    parser.SetStrictParsing()
    passive_block_index = \
        parser.AddModels("bindings/pydairlib/franka/urdf/passive_block.sdf")[0]
    active_block_index = \
        parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")[0]
    offset = RigidTransform(np.array([0.0, 0.0, 0.0]))
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base", passive_block_index), offset)
    plant.Finalize()
    # find the friction coefficient of the active block
    active_block_geometry_id = plant.GetCollisionGeometriesForBody(plant.GetBodyByName('active_block'))[0]
    collision_props = scene_graph.model_inspector().GetProximityProperties(active_block_geometry_id)
    friction_properties = collision_props.GetProperty('material', 'coulomb_friction')
    mu = friction_properties.dynamic_friction()

    plant_context = plant.CreateDefaultContext()
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(builder, plant, meshcat)
    controller = builder.AddSystem(Controller(plant, plant_context))
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port())

    state_logger = builder.AddSystem(VectorLogSink(plant.get_state_output_port().size()))
    input_logger = builder.AddSystem(VectorLogSink(controller.get_output_port().size()))
    builder.Connect(plant.get_state_output_port(), state_logger.get_input_port())
    builder.Connect(controller.get_output_port(), input_logger.get_input_port())
    diagram = builder.Build()

    meshcat.SetCameraPose(np.array([0, -3.0, 0.5]), np.array([0, 0.0, 0.0]))
    simulator = Simulator(diagram)
    # active x, passive: x, z, theta
    plant.SetPositions(diagram.GetMutableSubsystemContext(plant,
                                                          simulator.get_mutable_context()),
                       np.array([0.0, 0.0, 0.1, 0]))
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    def monitor(context):
        '''
        Monitors the simulation for episode end conditions.
        '''
        plant_context = plant.GetMyContextFromRoot(context)
        state = plant.GetOutputPort("state").Eval(plant_context)
        if state[2] < 0.0:
            print("Block Fell")
            return EventStatus.ReachedTermination(diagram, "block_fell")
        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)

    meshcat.StartRecording()
    sim_time = 40.0
    sim_samples = np.linspace(0, sim_time, int(1000 * sim_time))
    target = np.zeros((sim_samples.shape[0], 2))
    target[np.where(sim_samples// 10 % 2 == 0)] = np.array([0.0, -0.4])
    target[np.where(sim_samples// 10 % 2 != 0)] = np.array([0.0, 0.4])
    try:
        simulator.AdvanceTo(sim_time)
    except KeyboardInterrupt: # allow stopping early
        pass
    finally:

        meshcat.StopRecording()
        meshcat.PublishRecording()

        state_log_data = state_logger.GetLog(state_logger.GetMyContextFromRoot(simulator.get_context()))
        input_log_data = input_logger.GetLog(input_logger.GetMyContextFromRoot(simulator.get_context()))
        plot = plot_styler.PlotStyler(nrows=3)
        plot.plot(state_log_data.sample_times().T, state_log_data.data().T[:, :2], subplot_index=0, ylabel="Position (m)")
        plot.plot(sim_samples, target, subplot_index=0)
        plot.plot(state_log_data.sample_times().T, state_log_data.data().T[:, 4:6], subplot_index=1, ylabel="Velocity (m/s)")
        plot.plot(input_log_data.sample_times().T, input_log_data.data().T[:], subplot_index=2, ylabel="Input (Nm)", xlabel="Time (s)")
        if save:
            plot.save_fig(results_folder + 'plots/' + ('%.2f' % mu).replace('.', '') + '_' + method + '.png')
            with open(results_folder + 'meshcat_visualization/' + '%.2f' % mu + '_' + method + '.html', 'w') as f:
                f.write(meshcat.StaticHtml())
        plt.show()



if __name__ == '__main__':
    main()
