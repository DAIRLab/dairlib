from pydairlib.common import FindResourceOrThrow
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import pydrake.geometry
import numpy as np
from pydrake.lcm import DrakeLcm
import pdb

# Python visualization test to draw a single state
def main():
    lcm = DrakeLcm()

    builder = pydrake.systems.framework.DiagramBuilder()
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, 0)
    pydrake.multibody.parsing.Parser(plant).AddModelFromFile(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"))
    plant.Finalize()    

    pydrake.geometry.ConnectDrakeVisualizer(builder=builder,
        scene_graph=scene_graph)
    diagram = builder.Build()    


    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    state = plant_context.get_continuous_state_vector().get_mutable_value()
    state[0:plant.num_positions()] = np.random.random_sample(plant.num_positions())

    pydrake.geometry.DispatchLoadMessage(scene_graph=scene_graph, lcm=lcm,
        role=pydrake.geometry.Role.kIllustration)

    diagram.Publish(diagram_context)
    ids = scene_graph.model_inspector().GetAllGeometryIds()

    pdb.set_trace()

    # scene_graph.world_frame_id()


if __name__ == "__main__":
    main()
