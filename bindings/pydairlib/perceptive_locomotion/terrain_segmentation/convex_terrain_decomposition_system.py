import cv2
import numpy as np
from grid_map import GridMap

from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet
from pydairlib.geometry.polygon_utils import ProcessTerrain2d


class ConvexTerrainDecompositionSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_safe_terrain = self.DeclareAbstractInputPort(
            "terrain_segmentation", Value(GridMap())
        ).get_index()

        self.foothold_output_port = self.DeclareAbstractOutputPort(
            name="safe_footholds",
            alloc=lambda: Value(ConvexPolygonSet([])),
            calc=self.calc
        )

    def calc(self, context: Context, out: Value) -> None:
        # Get the safe terrain segmentation grid map
        grid = self.get_input_port(
            self.input_port_safe_terrain
        ).Eval(context)
        resolution = grid.getResolution()

        safe_terrain_image = (255 * grid['segmentation']).astype(np.uint8)

        nonconvex_safe_regions, hierarchy = cv2.findContours(
            safe_terrain_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )

        if hierarchy is None:
            return

        hierarchy = hierarchy.squeeze()

        # vector is an outer contour if is has no parent
        def is_outer_contour(hierarchy_vector):
            return hierarchy_vector[-1] < 0

        polygons_with_holes = []
        for i, boundary in enumerate(nonconvex_safe_regions):
            if is_outer_contour(hierarchy[i]):
                boundary_scaled = resolution * boundary.astype(float).squeeze().T
                polygon = (boundary_scaled, [])
                child_index = hierarchy[i][2]

                while child_index > 0:
                    hole_scaled = \
                        resolution * nonconvex_safe_regions[child_index].astype(float).\
                            squeeze().T
                    polygon[1].append(hole_scaled)
                    child_index = hierarchy[child_index][0]
                polygons_with_holes.append(polygon)

        polygons = ProcessTerrain2d(polygons_with_holes)
        out.set_value(ConvexPolygonSet(polygons))


