import pydrake.multibody.plant
import pydrake.systems.framework

class MultibodyPositionToGeometryPose(pydrake.systems.framework.LeafSystem):
    def __init__(self, plant: pydrake.multibody.plant.MultibodyPlant, input_multibody_state: bool = ...) -> None: ...
