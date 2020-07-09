from pydairlib.common import FindResourceOrThrow
from pydairlib.multibody import MultiposeVisualizer
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import numpy as np

# Python visualization test to draw a single state
def main():
    num_poses = 3
    alpha = np.array([.1, .3, 1])
    visualizer = MultiposeVisualizer(FindResourceOrThrow(
        "examples/Cassie/urdf/cassie_v2.urdf"), num_poses, alpha, "")
    # Create a plant just to know number of positions (also as a pydrake test)
    builder = pydrake.systems.framework.DiagramBuilder()
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, 0)
    pydrake.multibody.parsing.Parser(plant).AddModelFromFile(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"))
    plant.Finalize()
    visualizer.DrawPoses(np.random.rand(plant.num_positions(),
        num_poses))

if __name__ == "__main__":
    main()
