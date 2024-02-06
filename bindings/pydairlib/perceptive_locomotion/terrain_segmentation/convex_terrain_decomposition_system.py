import cv2
import time
import numpy as np
from skspatial.objects import Plane

from grid_map import GridMap, InterpolationMethods, InpaintWithMinimumValues

from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet

try:
    from pydairlib.geometry.polygon_utils import ProcessTerrain2d
except ImportError:
    print('ERROR: when built against ros, you need to source the ros environment before '
          'importing from polygon_utils, or python may not be able to find some necessary libraries. '
          'Please source ros and try again')
    exit(0)

import matplotlib.pyplot as plt


def plot_polygon(verts, linestyle='solid', color='color'):
    # plot a polygon (for debugging)
    assert(verts.shape[0] == 2)
    tmp = np.vstack((verts.T, verts[:, 0]))
    plt.plot(-tmp[:, 1], tmp[:, 0], linestyle=linestyle, color=color)


def plot_polygon_with_holes(poly):
    plot_polygon(poly[0], linestyle='solid', color='black')
    for p in poly[1]:
        plot_polygon(p, linestyle='dashed', color='grey')


def plot_polygons_with_holes(polys):
    for p in polys:
        plot_polygon_with_holes(p)
    plt.show()


class ConvexTerrainDecompositionSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_safe_terrain = self.DeclareAbstractInputPort(
            "terrain_segmentation", Value(GridMap())
        ).get_index()

        self.foothold_output_port = self.DeclareAbstractOutputPort(
            name="safe_footholds",
            alloc=lambda: Value(ConvexPolygonSet([])),
            calc=self.calc,
            prerequisites_of_calc={
                self.input_port_ticket(self.input_port_safe_terrain)
            }
        )

    def get_plane(self, elevation_map: GridMap, polygon: ConvexPolygon):
        verts3d = polygon.GetVertices().squeeze().transpose()
        for v in verts3d:
            v[-1] = elevation_map.atPosition(
                "elevation_inpainted", v[:2], InterpolationMethods.INTER_CUBIC
            )
        plane = None
        try:
            plane = Plane.best_fit(verts3d)
        except ValueError:
            import pdb; pdb.set_trace()

        return plane.normal, plane.point

    def calc(self, context: Context, out: Value) -> None:
        # Get the safe terrain segmentation grid map
        grid = self.EvalAbstractInput(
            context, self.input_port_safe_terrain
        ).get_value()

        safe_terrain_image = (255 * grid['segmentation']).astype(np.uint8)

        safe_regions, hierarchy = cv2.findContours(
            safe_terrain_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )

        if hierarchy is None:
            return

        hierarchy = hierarchy.squeeze()
        hierarchy = np.reshape(hierarchy, (-1, 4))

        # vector is an outer contour if is has no parent
        def is_outer_contour(hierarchy_vector):
            return hierarchy_vector[-1] < 0

        polygons = []
        for i, boundary in enumerate(safe_regions):
            boundary = np.fliplr(boundary.squeeze())
            if is_outer_contour(hierarchy[i]):
                boundary_points = np.zeros_like(boundary, dtype=float)
                for j in range(boundary_points.shape[0]):
                    boundary_points[j] = grid.getPosition(
                        index=boundary[j]
                    )

                polygon = (boundary_points.transpose(), [])
                child_index = hierarchy[i][2]

                while child_index > 0:
                    hole_boundary = np.fliplr(safe_regions[child_index].squeeze())
                    hole_points = np.zeros_like(
                        hole_boundary, dtype=float
                    )
                    for j in range(hole_points.shape[0]):
                        hole_points[j] = grid.getPosition(
                            index=hole_boundary[j]
                        )
                    polygon[1].append(hole_points.transpose())
                    child_index = hierarchy[child_index][0]
                polygons.append(polygon)

        convex_polygons = ProcessTerrain2d(polygons, 0.25)

        if polygons and not convex_polygons:
            return

        for polygon in convex_polygons:
            normal, point = self.get_plane(grid, polygon)
            polygon.SetPlane(normal, point)
        out.set_value(ConvexPolygonSet(convex_polygons))
