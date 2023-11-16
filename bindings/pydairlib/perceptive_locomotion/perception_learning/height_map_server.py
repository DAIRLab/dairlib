import numpy as np
from typing import Union
from dataclasses import dataclass

from pydrake.multibody.plant import (
    MultibodyPlant
)

from pydairlib.geometry.convex_polygon import ConvexPolygon

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

from pydairlib.systems.footstep_planning import Stance


@dataclass
class HeightMapOptions:
    nx: int = 20
    ny: int = 20
    resolution: float = 0.02


class HeightMapServer:
    """
        Uses the robot state and some controller information
        and calculates a height map in the stance frame
    """

    def __init__(self, terrain: Union[str, SquareSteppingStoneList], urdf: str,
                 map_opts: HeightMapOptions = HeightMapOptions()):
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

        stepping_stones = LoadSteppingStonesFromYaml(terrain) if isinstance(
            terrain, str
        ) else terrain

        self.convex_terrain_segments = \
            SquareSteppingStoneList.GetFootholdsWithMargin(
                stepping_stones.stones, 0.0
            )[0]

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
        self.contact_point = \
            0.5 * (
                    RightToeRear(self.plant)[0] + RightToeFront(self.plant)[0]
            ).ravel()
        self.contact_frame = {
            Stance.kLeft: LeftToeRear(self.plant)[1],
            Stance.kRight: RightToeRear(self.plant)[1]
        }

    def get_height_at_point(self, query_point: np.ndarray) -> float:
        zvals = []
        for seg in self.convex_terrain_segments:
            if seg.Get2dViolation(query_point) <= 0:
                A, b = seg.GetEqualityConstraintMatrices()
                z = b - A[:, :2] @ query_point[:2]
                zvals.append(z)
        if zvals:
            return np.max(zvals)
        else:
            return np.nan

    def query_height_in_stance_frame(self, xy: np.ndarray,
                                     robot_state: np.ndarray,
                                     stance: Stance) -> float:
        self.plant.SetPositionsAndVelocities(self.plant_context, robot_state)
        stance_pos = self.plant.CalcPointsPositions(
            self.plant_context,
            self.contact_frame[stance],
            self.contact_point,
            self.plant.world_frame()
        ).ravel()
        query_point = stance_pos + ReExpressBodyYawVector3InWorldFrame(
            plant=self.plant,
            context=self.plant_context,
            body_name="pelvis",
            vec=np.array([xy[0], xy[1], 0.0])
        )
        return self.get_height_at_point(query_point)

    def get_heightmap_3d(self, robot_state: np.ndarray, stance: Stance,
                         center: np.ndarray = None) -> np.ndarray:
        center = np.zeros((3,)) if center is None else center
        X, Y = np.meshgrid(self.xgrid + center[0], self.ygrid + center[1])

        # Heightmap has X axis along the 0 dimension while meshgrid has
        # X axis on the 1 dimensions
        Z = self.get_heightmap(robot_state, stance, center).transpose()

        return np.stack([X, Y, Z])

    def get_heightmap(self, robot_state: np.ndarray, stance: Stance,
                      center: np.ndarray = np.zeros((3,))) -> np.ndarray:
        self.plant.SetPositionsAndVelocities(self.plant_context, robot_state)
        stance_pos = self.plant.CalcPointsPositions(
            self.plant_context,
            self.contact_frame[stance],
            self.contact_point,
            self.plant.world_frame()
        ).ravel()
        heightmap = np.zeros((self.map_opts.nx, self.map_opts.ny))
        for i, x in enumerate(self.xgrid):
            for j, y in enumerate(self.ygrid):
                offset = ReExpressBodyYawVector3InWorldFrame(
                    plant=self.plant,
                    context=self.plant_context,
                    body_name="pelvis",
                    vec=np.array([x, y, 0.0])
                )
                query_point = stance_pos + center + offset
                heightmap[i, j] = self.get_height_at_point(query_point)

        return heightmap
