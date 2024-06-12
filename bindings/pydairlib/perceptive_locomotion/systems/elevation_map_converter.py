import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter, median_filter
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
from pydairlib.perceptive_locomotion.utils import CalcHeightMapInStanceFrame


from grid_map import GridMap, InterpolationMethods
from grid_map_filters import InpaintWithMinimumValues
#InterpolationMethods.INTER_NEAREST | INTER_LINEAR | INTER_CUBIC_CONVOLUTION | INTER_CUBIC

@dataclass
class ElevationMapOptions:
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
class ElevationMapQueryObject:
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
            self.height_map_server.get_height_map_in_stance_frame_from_inputs(
                self.context, query_point
            )
        return hmap

    def calc_z_stance_frame(self, query_point):
        if self.context is None:
            raise RuntimeError( 
                'Heightmap Queries are one-time use objects'
                'that must be set from inside the HeightMapServer. '
                'Context has not been set or is out-of-date')
        hmap = \
            self.height_map_server .get_z_in_stance_frame_from_inputs(
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


class ElevationMappingConverter(LeafSystem):
    """
        Uses the robot state and some controller information
        and calculates a height map in the stance frame
    """
    def __init__(
        self, urdf: str, map_opts: ElevationMapOptions = ElevationMapOptions()):

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

        # preallocate grids in local frame for faster lookup times
        self.xgrid = np.linspace(
            -self.map_opts.resolution * self.map_opts.nx / 2,
            self.map_opts.resolution * self.map_opts.nx / 2,
            self.map_opts.nx
        )
        self.ygrid = np.linspace(
            -(self.map_opts.resolution * self.map_opts.ny / 2),
            (self.map_opts.resolution * self.map_opts.ny / 2),
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

        self.input_port_indices['elevation'] = self.DeclareAbstractInputPort(
            'elevation', Value(GridMap())
        ).get_index()

        self.output_port_indices['query_object'] = self.DeclareAbstractOutputPort(
            name="height_map_stance_frame",
            alloc=lambda: Value(ElevationMapQueryObject()),
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

        grid_map = self.EvalAbstractInput(
            context, self.input_port_indices['elevation']).get_value()
        InpaintWithMinimumValues(grid_map, "elevation", "elevation_inpainted")

        return self.get_heightmap_3d(x, stance, grid_map, center)

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

        grid_map = self.EvalAbstractInput(
            context, self.input_port_indices['elevation']).get_value()
        InpaintWithMinimumValues(grid_map, "elevation", "elevation_inpainted")

        hmap_xyz = self.get_heightmap_3d_world_frame(x, stance, grid_map, center)

        return hmap_xyz

    def get_height_at_point(self, query_point: np.ndarray, grid_map) -> float:
        return grid_map.atPosition(
            'elevation_inpainted', query_point[:2], InterpolationMethods.INTER_NEAREST #InterpolationMethods.INTER_CUBIC_CONVOLUTION
        )

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
                                     stance: Stance,
                                     grid_map) -> float:
        stance_pos = self.stance_pos_in_world(robot_state, stance)
        query_point = stance_pos + ReExpressBodyYawVector3InWorldFrame(
            plant=self.plant,
            context=self.plant_context,
            body_name="pelvis",
            vec=np.array([xy[0], xy[1], 0.0])
        )
        return self.get_height_at_point(query_point, grid_map) - stance_pos[2]

    def get_heightmap_3d_world_frame(self, robot_state: np.ndarray,
                                     stance: Stance,
                                     grid_map: GridMap,
                                     center: np.ndarray = None):
        stance_pos = self.stance_pos_in_world(robot_state, stance)
        xyz = self.get_heightmap_3d(robot_state, stance, grid_map, center)
        for i in range(xyz.shape[1]):
            for j in range(xyz.shape[2]):
                xyz[:, i, j] = stance_pos + ReExpressBodyYawVector3InWorldFrame(
                    plant=self.plant,
                    context=self.plant_context,
                    body_name="pelvis",
                    vec=xyz[:, i, j]
                )
        return xyz

    def get_heightmap_3d(self, robot_state: np.ndarray,
                         stance: Stance,
                         grid_map: GridMap,
                         center: np.ndarray = None) -> np.ndarray:
        center = np.zeros((3,)) if center is None else center
        X, Y = np.meshgrid(self.xgrid + center[0], self.ygrid + center[1])

        # Heightmap has X axis along the 0 dimension while meshgrid has
        # X axis on the 1 dimensions
        Z = self.get_heightmap(robot_state, stance, grid_map, center).transpose()

        return np.stack([X, Y, Z])

    def get_heightmap(self, robot_state: np.ndarray,
                      stance: Stance,
                      grid_map: GridMap,
                      center: np.ndarray = np.zeros((3,))) -> np.ndarray:
        stance_pos = self.stance_pos_in_world(robot_state, stance)

        heightmap = CalcHeightMapInStanceFrame(
            grid_map=grid_map,
            layer="elevation_inpainted",
            plant=self.plant,
            plant_context=self.plant_context,
            floating_base_body_name="pelvis",
            stance_pos=stance_pos,
            center=center,
            xgrid_stance_frame=self.xgrid,
            ygrid_stance_frame=self.ygrid
        )

        coordinates = np.argwhere(~np.isnan(heightmap)) # Coordinates of known values
        values = heightmap[~np.isnan(heightmap)] # Known values from the map
        grid_x, grid_y = np.mgrid[0:heightmap.shape[0], 0:heightmap.shape[1]] # Grid of coordinates for interpolation
        filled_map = griddata(coordinates, values, (grid_x, grid_y), method='nearest') # Interpolate using linear, nearest or cubic

        heightmap = median_filter(filled_map, size=3)
        heightmap = np.nan_to_num(heightmap, nan=-1)
        
        return heightmap