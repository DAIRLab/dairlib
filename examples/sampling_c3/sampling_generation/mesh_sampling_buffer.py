import math
import trimesh as tm
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points, unary_union
from shapely.geometry import LineString
import matplotlib.pyplot as plt

filename = f'examples/sampling_c3/urdf/push_t_white.obj'
mesh = tm.load_mesh(filename)
bounds = mesh.bounds
print(bounds)

z_height = bounds[0, 2] + 0.03
print(z_height)
section = mesh.section(plane_origin=[0, 0, z_height], plane_normal=[0,0,1])

paths, _ = section.to_2D()

polygons = paths.polygons_full
original_graph = paths.polygons_full[0].exterior
buffer_distance = 0.03
unified_polygons = unary_union([poly.exterior for poly in polygons])
buffered_polygons = unified_polygons.buffer(buffer_distance)
graph = buffered_polygons.exterior

x, y = graph.xy
# plt.scatter(x, y, color='blue')
a, b = original_graph.xy
plt.scatter(a, b, color='red')

num_points = 100
distances= np.linspace(0, graph.length, num_points, endpoint=False)
points = [graph.interpolate(distance) for distance in distances]
x_coords = [point.x for point in points]
y_coords = [point.y for point in points]
plt.scatter(x_coords, y_coords, color='green')

num_vertices = len(x)
print(num_vertices)

paths.show()