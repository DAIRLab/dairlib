from dataclasses import dataclass
from os import path

from pydrake.all import (
    Simulator,
    Diagram,
    Context,
)

from pydairlib.common import (
    FindResourceOrThrow
)

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"


@dataclass
class CassieFootstepControllerEnvironmentOptions:
    terrain_yaml: str = path.join(
        perception_learning_base_folder,
        'params/terrain.yaml'
    )


class CassieFootstepControllerEnvironment:

    def __init__(self):
        pass
