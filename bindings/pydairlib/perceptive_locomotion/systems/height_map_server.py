import numpy as np
from typing import Union
from dataclasses import dataclass

from pydrake.multibody.plant import (
    MultibodyPlant
)

from pydrake.systems.all import (
    LeafSystem,
    InputPort,
    Context,
    Value,
    State
)

from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet
from pydairlib.geometry.meshcat_utils import PlotColoredSurface

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
    nx: int = 64
    ny: int = 64
    resolution: float = 0.025
    meshcat = None


# In order to request an up-to-date heightmap from within the
# LQR controller, we need to be able to pass the query point
# from the LQR controller back to the height map.
#
# The query object lets us do that without creating an algebraic
# loop, similar to Drake's SceneGraph implementation
class HeightMapQueryObject:
    def __init__(self):
        self.height_map_server = None
        self.context = None

    def set(self, context: Context, server):
        self.height_map_server = server
        self.context = context

    def unset(self):
        self.context = None
        self.height_map_server = None

    def calc_height_map_stance_frame(self, query_point):
        if self.context is None:
            raise RuntimeError(
                'Heightmap Queries are one-time use objects'
                'that must be set from inside the HeightMapServer. '
                'Context has not been set or is out-of-date')
        hmap = \
            self.height_map_server .get_height_map_in_stance_frame_from_inputs(
                self.context, query_point
            )
        return hmap
    
    def calc_height_map_world_frame(self, query_point):
        # get height map in world frame
        if self.context is None:
            raise RuntimeError(
                'Heightmap Queries are one-time use objects'
                'that must be set from inside the HeightMapServer. '
                'Context has not been set or is out-of-date')
        # grab original hmap in world frame
        hmap = \
            self.height_map_server .get_height_map_in_world_frame_from_inputs(
                self.context, query_point
            )
        return hmap

    def plot_surface(self, path, X, Y, Z, rgba):
        if self.height_map_server.map_opts.meshcat is not None:
            self.height_map_server.map_opts.meshcat.PlotSurface(
                path, X, Y, Z, rgba
            )
            self.height_map_server.map_opts.meshcat.Flush()

    def plot_colored_surface(self, path, X, Y, Z, R, G, B):
        if self.height_map_server.map_opts.meshcat is not None:
            PlotColoredSurface(path,
                self.height_map_server.map_opts.meshcat,
                X, Y, Z, R, G, B, False, 1.0
            )
            self.height_map_server.map_opts.meshcat.Flush()


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
        self.terrain = ConvexPolygonSet(self.convex_terrain_segments)

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

        self.input_port_indices = dict()
        self.output_port_indices = dict()

        self.input_port_indices['fsm'] = self.DeclareVectorInputPort('fsm', 1).get_index()
        self.input_port_indices['x'] = self.DeclareVectorInputPort(
            'x', self.plant.num_positions() +
                 self.plant.num_velocities() +
                 self.plant.num_actuators() + 4
        ).get_index()

        self.output_port_indices['query_object'] = self.DeclareAbstractOutputPort(
            name="height_map_stance_frame",
            alloc=lambda: Value(HeightMapQueryObject()),
            calc=self.output_query_object
        )

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def output_query_object(self, context: Context, out: Value):
        out.get_mutable_value().set(context, self)

    def get_height_map_in_stance_frame_from_inputs(
            self, context: Context, center: np.ndarray
        ) -> np.ndarray:

        fsm = self.EvalVectorInput(
            context, self.input_port_indices['fsm']
        ).value().ravel()[0]
        fsm = int(fsm)
        x = self.EvalVectorInput(
            context, self.input_port_indices['x']
        ).value().ravel()[:self.plant.num_positions() + self.plant.num_velocities()]

        stance = Stance.kLeft if fsm == 0 or fsm == 3 else Stance.kRight

        if self.map_opts.meshcat is not None:
            hmap_xyz = self.get_heightmap_3d_world_frame(x, stance, center)
            # self.map_opts.meshcat.PlotSurface(
            #     "hmap", hmap_xyz[0], hmap_xyz[1], hmap_xyz[2],
            #     Rgba(0, 0, 1, 0.5)
            # )
            self.map_opts.meshcat.Flush()
        return self.get_heightmap_3d(x, stance, center)
    
    def get_height_map_in_world_frame_from_inputs(
            self, context: Context, center: np.ndarray
        ) -> np.ndarray:

        fsm = self.EvalVectorInput(
            context, self.input_port_indices['fsm']
        ).value().ravel()[0]
        fsm = int(fsm)
        x = self.EvalVectorInput(
            context, self.input_port_indices['x']
        ).value().ravel()[:self.plant.num_positions() + self.plant.num_velocities()]

        stance = Stance.kLeft if fsm == 0 or fsm == 3 else Stance.kRight

        hmap_xyz = self.get_heightmap_3d_world_frame(x, stance, center)
        
        return hmap_xyz

    def get_height_at_point(self, query_point: np.ndarray) -> float:
        return self.terrain.CalcHeightOfPoint(query_point)

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
                    vec=np.array([x, y, 0.0]) + center
                )
                query_point = stance_pos + offset
                heightmap[i, j] = self.get_height_at_point(query_point) - stance_pos[2]

        return heightmap
