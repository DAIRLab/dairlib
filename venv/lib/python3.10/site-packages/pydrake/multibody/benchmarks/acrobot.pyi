import pydrake.geometry
import pydrake.multibody.plant

class AcrobotParameters:
    def __init__(self) -> None: ...

def MakeAcrobotPlant(default_parameters: AcrobotParameters, finalize: bool, scene_graph: pydrake.geometry.SceneGraph = ...) -> pydrake.multibody.plant.MultibodyPlant: ...
