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
)

class CubePusher(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce]]
        self.DeclareAbstractOutputPort(
            "applied_force", lambda: forces_cls(), self.CalcOutput
        )
        self.plant = plant

    def CalcOutput(self, context, output):
        forces = []
        force = ExternallyAppliedSpatialForce()
        force.body_index = self.plant.GetBodyByName("active_block").index()
        force.F_Bq_W = SpatialForce(
            tau=np.array([0, 0, 0]),
            f=[4 * np.sin(1.0 * context.get_time()), 0, 0],
                )
        forces.append(force)
        output.set_value(forces)
def main():


    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0005)
    parser = Parser(plant)
    parser.AddModels("bindings/pydairlib/franka/urdf/passive_block.sdf")
    parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")
    offset = RigidTransform(np.array([0.0, 0.0, 0.25]))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("center"))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), offset)
    plant.Finalize()
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(builder, plant, meshcat)
    pusher = builder.AddSystem(CubePusher(plant))
    builder.Connect(pusher.get_output_port(), plant.get_applied_spatial_force_input_port())
    diagram = builder.Build()
    meshcat.SetCameraPose(np.array([0, -3.0, 0.5]), np.array([0, 0.0, 0.0]))
    simulator = Simulator(diagram)
    context = simulator.get_context()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(60.0)

if __name__ == '__main__':
    main()