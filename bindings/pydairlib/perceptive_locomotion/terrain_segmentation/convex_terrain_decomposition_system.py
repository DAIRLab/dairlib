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
from pydairlib.geometry.polygon_utils import ProcessTerrain2d, GetAcdComponents

import matplotlib.pyplot as plt


def plot_polygon(verts, linestyle='solid', color='black'):
    # plot a polygon (for debugging)
    assert(verts.shape[0] == 2)
    tmp = np.vstack((verts.T, verts[:, 0]))
    plt.plot(-tmp[:, 1], tmp[:, 0], linestyle=linestyle, linewidth=2.5, color=color)


def plot_polygon_with_holes(poly):
    plot_polygon(poly[0], linestyle='solid', color='black')
    for p in poly[1]:
        plot_polygon(p, linestyle='solid', color='black')


def plot_polygons_with_holes(polys):
    for p in polys:
        plot_polygon_with_holes(p)


def remove_collinear(boundary):
    len = boundary.shape[0]
    keep_idx = []
    for i in range(len):
        prev_idx = i - 1 if i > 0 else len - 1
        next_idx = (i + 1) % len
        p = boundary[prev_idx, :]
        c = boundary[i, :]
        n = boundary[next_idx, :]
        if p[0] == c[0] == n[0] or p[1] == c[1] == n[1]:
            continue
        else:
            keep_idx.append(i)
    return boundary[keep_idx, :]


def get_polygons_by_contour_extraction(mask: np.ndarray, grid: GridMap):
    safe_regions, hierarchy = cv2.findContours(
        mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )

    if hierarchy is None:
        return []

    hierarchy = hierarchy.squeeze()
    hierarchy = np.reshape(hierarchy, (-1, 4))

    # vector is an outer contour if is has no parent
    def is_outer_contour(hierarchy_vector):
        return hierarchy_vector[-1] < 0

    polygons = []
    for i, boundary in enumerate(safe_regions):
        boundary = np.fliplr(boundary.squeeze())
        boundary = remove_collinear(boundary)
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
                hole_boundary = remove_collinear(hole_boundary)
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

    return polygons


class ConvexTerrainDecompositionSystem(LeafSystem):

    def __init__(self, profiling=None):
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
        self.profiling = profiling
        self.debug = False
        self.debug_info = {}
        self.acd_thresh = 0.25

    def get_plane(self, elevation_map: GridMap, polygon: ConvexPolygon):
        verts3d = None
        try:
            verts3d = polygon.GetVertices().squeeze().transpose()
        except RuntimeError:
            print("error getting vertices")
            return None, None

        for v in verts3d:
            v[-1] = elevation_map.atPosition(
                "elevation_inpainted", v[:2], InterpolationMethods.INTER_CUBIC
            )
        plane = None
        try:
            plane = Plane.best_fit(verts3d)
        except ValueError:
            print("error fitting plane to points:")
            print(verts3d)
            return None, None
        
        if np.isnan(plane.normal).any() or np.isnan(plane.point).any() or \
           np.isinf(plane.normal).any() or np.isinf(plane.point).any():
            print("Found invalid plane parameters: ")
            print(f'normal: {plane.normal}, point: {plane.point}')
            return None, None
        
        return plane.normal, plane.point

    def calc_convex_polygons(self, grid: GridMap):
        safe_terrain_image = (255 * grid['segmentation']).astype(np.uint8)
        
        polygons = get_polygons_by_contour_extraction(
            safe_terrain_image, grid
        )
        
        if not polygons:
            return None
        
        return ProcessTerrain2d(polygons, self.acd_thresh)

    def calc(self, context: Context, out: Value) -> None:
        # Get the safe terrain segmentation grid map
        start = time.time()
        grid = self.EvalAbstractInput(
            context, self.input_port_safe_terrain
        ).get_value()
        
        safe_terrain_image = (255 * grid['segmentation']).astype(np.uint8)
    
        polygons = get_polygons_by_contour_extraction(
            safe_terrain_image, grid
        )
        
        if not polygons:
            return None
        
        convex_polygons = ProcessTerrain2d(polygons, self.acd_thresh)
        
        end_convexity = time.time()

        if not convex_polygons:
            if self.profiling:
                self.profiling['decomposition'].append(end_convexity - start)
                self.profiling['plane_fitting'].append(0)
                self.profiling['num_polygons'].append(0)
            return

        valid_polys = []
        for polygon in convex_polygons:
            normal, point = self.get_plane(grid, polygon)
            if normal is not None:
                polygon.SetPlane(normal, point)
                valid_polys.append(polygon)

        end_plane_fitting = time.time()
        out.set_value(ConvexPolygonSet(valid_polys))

        if self.debug:
            self.debug_info['unprocessed_polygons'] = polygons
            self.debug_info['segmentation'] = safe_terrain_image
            self.debug_info['acd_components'] = GetAcdComponents(polygons, self.acd_thresh)
            self.debug_info['convex_polygons'] = convex_polygons

        if self.profiling:
            self.profiling['decomposition'].append(end_convexity - start)
            self.profiling['plane_fitting'].append(end_plane_fitting - end_convexity)
            self.profiling['num_polygons'].append(len(convex_polygons))
