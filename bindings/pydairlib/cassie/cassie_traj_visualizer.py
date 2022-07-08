from pydrake.multibody.plant import MultibodyPlant
from pydrake.geometry import SceneGraph
from pydairlib.multibody.plant_traj_visualizer import PlantTrajVisualizer
from pydairlib.cassie.cassie_utils import AddCassieMultibody


class CassieTrajVisualizer(PlantTrajVisualizer):
    def __init__(self, traj):
        urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
        plant = MultibodyPlant(0.0)
        scene_graph = SceneGraph()
        AddCassieMultibody(plant, scene_graph, True, urdf, False, False)
        super().__init__(plant, scene_graph, traj)
