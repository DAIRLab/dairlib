import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ContactVisualizer,
    DiagramBuilder,
    ExternallyAppliedSpatialForce,
    LeafSystem,
    List,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    Simulator,
    SpatialForce,
    StartMeshcat,
    RigidTransform,
    Value,
    InitializeAutoDiff,
    ModelInstanceIndex
)

from c3 import *


class Controller(LeafSystem):
    def __init__(self, plant, plant_context):
        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce]]
        self.DeclareVectorOutputPort(
            "controller_output", 1, self.CalcOutput
        )
        self.DeclareVectorInputPort("system_state", plant.num_positions() + plant.num_velocities())
        self.plant = plant
        self.plant_context = plant_context

        lcs_builer = DiagramBuilder()
        self.plant_for_lcs, self.scene_graph_for_lcs = AddMultibodyPlantSceneGraph(
            lcs_builer, 0.0)
        lcs_parser = Parser(self.plant_for_lcs)
        self.passive_block_index = lcs_parser.AddModels(
            "bindings/pydairlib/franka/urdf/passive_block.sdf")[0]
        self.active_block_index = \
        lcs_parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")[
            0]
        self.plant_for_lcs.WeldFrames(self.plant_for_lcs.world_frame(),
                                      self.plant_for_lcs.GetFrameByName("base",
                                                                        self.passive_block_index))
        self.plant_for_lcs.Finalize()
        lcs_diagram = lcs_builer.Build()

        diagram_context = lcs_diagram.CreateDefaultContext()
        self.context_for_lcs = lcs_diagram.GetMutableSubsystemContext(
            self.plant_for_lcs, diagram_context)
        self.plant_ad = self.plant_for_lcs.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()

        passive_block_contact_points = self.plant_for_lcs.GetCollisionGeometriesForBody(
            self.plant_for_lcs.GetBodyByName("passive_block",
                                             self.passive_block_index))
        active_block_contact_points = self.plant_for_lcs.GetCollisionGeometriesForBody(
            self.plant_for_lcs.GetBodyByName("active_block",
                                             self.active_block_index))
        self.contact_geoms = list()
        for geom_id in active_block_contact_points:
            # contact_geoms.append([geom_id, passive_block_contact_points[0]])
            self.contact_geoms.append((geom_id, passive_block_contact_points[0]))
        self.num_friction_directions = 2
        self.mu = [0.4]
        self.dt = 0.1
        self.N = 5
        self.contact_model = ContactModel.kAnitescu



    def CalcOutput(self, context, output):
        x = self.EvalVectorInput(context, 0)
        x_u = np.hstack((x.value(), np.array([0])))
        x_u_ad = InitializeAutoDiff(x_u)
        self.plant_for_lcs.SetPositionsAndVelocities(self.context_for_lcs, x.value())
        self.plant_for_lcs.get_actuation_input_port().FixValue(self.context_for_lcs, np.array([0]))
        self.plant_ad.get_actuation_input_port().FixValue(self.context_ad, x_u_ad[-1])
        lcs = LinearizePlantToLCS(self.plant_for_lcs, self.context_for_lcs,
                                  self.plant_ad, self.context_ad, self.contact_geoms,
                                  self.num_friction_directions, self.mu, self.dt, self.N,
                                  self.contact_model)
        # import pdb; pdb.set_trace()
        # print(lcs.A_[0])
        print(lcs.D_[0].shape)
        # import pdb; pdb.set_trace()
        output.set_value(np.array([5 * np.sin(1.0 * context.get_time())]))


def main():
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0005)
    parser = Parser(plant)
    passive_block_index = \
    parser.AddModels("bindings/pydairlib/franka/urdf/passive_block.sdf")[0]
    active_block_index = \
    parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")[0]
    offset = RigidTransform(np.array([0.0, 0.0, 0.0]))
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base", passive_block_index), offset)
    # plant.WeldFrames(plant.world_frame(),
    #                  plant.GetFrameByName("base", active_block_index))
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(builder, plant, meshcat)
    controller = builder.AddSystem(Controller(plant, plant_context))
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port())
    diagram = builder.Build()

    meshcat.SetCameraPose(np.array([0, -3.0, 0.5]), np.array([0, 0.0, 0.0]))
    simulator = Simulator(diagram)
    context = simulator.get_context()
    plant.SetPositions(diagram.GetMutableSubsystemContext(plant,
                                                          simulator.get_mutable_context()),
                       np.array([0, 0.25, 0.0, 0]))  # x, z, theta
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(60.0)


if __name__ == '__main__':
    main()
