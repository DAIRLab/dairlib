import numpy as np
from typing import Union
from dataclasses import dataclass

from pydrake.multibody.plant import (
    MultibodyPlant
)

from pydrake.geometry.all import Meshcat

from pydrake.systems.all import (
    LeafSystem,
    Context,
    Value,
    State
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
    nx: int = 30
    ny: int = 30
    resolution: float = 0.02


class HeightMapServer(LeafSystem):
    """
        Uses the robot state and some controller information
        and calculates a height map in the stance frame
    """

    def __init__(self, terrain: Union[str, SquareSteppingStoneList], urdf: str,
                 map_opts: HeightMapOptions = HeightMapOptions()):

        super().__init__()
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

        self.DeclareAbstractOutputPort(
            name="height_map_stance_frame",
            alloc=lambda: Value(np.zeros((3, self.map_opts.nx, self.map_opts.ny))),
            calc=self.output_heightmap
        )

    def output_heightmap(self, context: Context, hmap: np.ndarray)-> None:
        pass

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

    def stance_pos_in_world(self, x: np.ndarray, stance: Stance) -> np.ndarray:
        self.plant.SetPositionsAndVelocities(self.plant_context, x)
        return self.plant.CalcPointsPositions(
            self.plant_context,
            self.contact_frame[stance],
            self.contact_point,
            self.plant.world_frame()
        ).ravel()

    def query_height_in_stance_frame(self, xy: np.ndarray,
                                     robot_state: np.ndarray,
                                     stance: Stance) -> float:
        stance_pos = self.stance_pos_in_world(robot_state, stance)
        query_point = stance_pos + ReExpressBodyYawVector3InWorldFrame(
            plant=self.plant,
            context=self.plant_context,
            body_name="pelvis",
            vec=np.array([xy[0], xy[1], 0.0])
        )
        return self.get_height_at_point(query_point) - stance_pos[2]

    def get_heightmap_3d_world_frame(self, robot_state: np.ndarray,
                                     stance: Stance, center: np.ndarray = None):
        stance_pos = self.stance_pos_in_world(robot_state, stance)
        xyz = self.get_heightmap_3d(robot_state, stance, center)
        for i in range(xyz.shape[1]):
            for j in range(xyz.shape[2]):
                xyz[:, i, j] = stance_pos + ReExpressBodyYawVector3InWorldFrame(
                    plant=self.plant,
                    context=self.plant_context,
                    body_name="pelvis",
                    vec=xyz[:, i, j]
                )
        return xyz

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
        stance_pos = self.stance_pos_in_world(robot_state, stance)
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
                heightmap[i, j] = self.get_height_at_point(query_point) - stance_pos[2]

        return heightmap


class HeightMapVisualizer(LeafSystem):

    def __init__(self, meshcat: Meshcat, hmap_shape):
        super().__init__()
        self.meshcat = Meshcat
        self.input_port_indices = dict()
        self.input_port_indices['height_map'] = self.DeclareAbstractInputPort(
            "height_map",
            model_value=Value(np.ndarray(shape=(3,20,20)))
        ).get_index()

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0/30.0,
            offset=0.0,
            update=self.UpdateVisualization
        )

    def UpdateVisualization(self, context: Context, state: State):
        hmap = self.get_input_port().Eval(context).get_value()
        self.meshcat.PlotSurface(
            "hmap", hmap[0], hmap[1], hmap[2]
        )
