from dataclasses import dataclass
import numpy as np

from pydrake.multibody.plant import (
    MultibodyPlant
)

from pydairlib.geometry.convex_foothold import ConvexFoothold

from pydairlib.cassie.cassie_utils import (
    AddCassieMultibody,
    RightToeFront,
    RightToeRear,
    LeftToeFront,
    LeftToeRear,
)

from pydairlib.multibody import (
    SquareSteppingStoneList,
    LoadSteppingStonesFromYaml,
    ReExpressWorldVector3InBodyYawFrame,
    ReExpressBodyYawVector3InWorldFrame,
)

from pydairlib.perceptive_locomotion.controllers import Stance


@dataclass
class HeightMapOptions:
    nx: int = 20
    ny: int = 20
    resolution: float = 0.05


class HeightMapServer:
    """
        Uses the robot state and some controller information
        and calculates a height map in the stance frame
    """

    def __init__(self, terrain_yaml: str, urdf: str, map_opts:
    HeightMapOptions = HeightMapOptions()):
        self.map_opts = map_opts
        self.plant = MultibodyPlant(0.0)
        _ = AddCassieMultibody(
            self.plant,
            None,
            True,
            urdf,
            True,
            False,
            True
        )
        self.plant.Finalize()
        self.plant_context = self.plant.CreateDefaultContext()

        stepping_stones = LoadSteppingStonesFromYaml(terrain_yaml)
        self.convex_terrain_segments = \
            SquareSteppingStoneList.GetFootholdsWithMargin(
                stepping_stones.stones
            )

        # preallocate grids in local frame for faster lookup times
        self.xgrid = np.linspace(
            -self.map_opts.resolution * self.map_opts.nx / 2,
            self.map_opts.resolution * self.map_opts.nx / 2,
            self.map_opts.nx
        )
        self.ygrid = np.linspace(
            -self.map_opts.resolution * self.map_opts.ny / 2,
            self.map_opts.resolution * self.map_opts.ny / 2,
            self.map_opts.ny
        )

    def get_height_at_point(self, query_point: np.ndarray) -> float:
        zvals = []
        for seg in self.convex_terrain_segments:
            if seg.Get2dViolation(query_point) <= 0:
                A, b = seg.GetEqualityConstraintMatrices()
                z = b - A[:, :2] @ query_point[:2]
                zvals.append(z)
        if zvals:
            return max(zvals)
        else:
            return np.nan

    def get_heightmap(self, robot_state: np.ndarray, stance: Stance):
        self.plant.SetPositionsAndVelocities(self.plant_context, robot_state)
