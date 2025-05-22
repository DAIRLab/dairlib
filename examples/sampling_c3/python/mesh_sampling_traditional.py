import math
import trimesh as tm
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points, unary_union
from shapely.geometry import LineString
import matplotlib.pyplot as plt

filename = f'examples/sampling_c3/urdf/milk_bsdf_mesh.obj'
mesh = tm.load_mesh(filename)
bounds = mesh.bounds
print(bounds)

z_height = bounds[0, 2] + 0.05
n_points = 200

print(bounds[0, 2], z_height)
x_min, y_min = -bounds[0,0], bounds[0, 1]
x_max, y_max = -bounds[1,0], bounds[1, 1]
print(x_min, y_min, x_max, y_max)

x_rand, y_rand = np.random.uniform(x_min, x_max, n_points), np.random.uniform(x_min, x_max, n_points)
z_rand = np.full(n_points, z_height)
points = np.vstack((x_rand, y_rand, z_rand)).T
points_2d = np.vstack((x_rand, y_rand)).T

in_mesh = mesh.contains(points)

section = mesh.section(plane_origin=[0, 0, z_height], plane_normal=[0,0,1])

paths, _ = section.to_2D()
random_points = paths.sample(n_points)
all_polys = paths.polygons_full
# minx, miny, maxx, maxy = all_polys.bounds

# print(all_polys)
mask = np.array([any(poly.contains(Point(x, y)) for poly in all_polys) for x, y in points_2d])
print(type(mask))
print(mask.shape)
flat_mask = mask.flatten()
print(flat_mask.shape)
inside_points = points_2d[flat_mask]
print(inside_points)

plt.scatter(inside_points[:, 0], inside_points[:,1])

unified_polys = unary_union([poly.exterior for poly in all_polys])
inside_points_shapely = [Point(x, y) for x, y in inside_points]
nearest_points_on_boundary = [nearest_points(Point(x, y), unified_polys)[1] for x, y in inside_points]
nearest_coords = np.array([point.coords[0] for point in nearest_points_on_boundary])
directions = nearest_coords - inside_points
norms = np.linalg.norm(directions, axis=1, keepdims=True)
norms[norms == 0] = 1
unit_dirs = directions / norms
offset = .0195 + .03
new_points = inside_points + offset * unit_dirs + directions
distances = np.array([unified_polys.distance(Point(x, y)) for x, y in new_points])
mask_far = distances >= .025
filtered_new_points = new_points[mask_far]
plt.scatter(filtered_new_points[:, 0], filtered_new_points[:, 1], c='r', marker='x')
N = inside_points.shape[0]
labels = np.arange(1, N+1)
for i, (x, y) in enumerate(inside_points):
    plt.text(x, y, str(labels[i]))
for i, (x, y) in enumerate(new_points):
    plt.text(x, y, str(labels[i]))
paths.show()

# dx = []
# dy = []

# for x, y in inside_points:
#     point = Point(x, y)
#     nearest_dist_x = 100
#     nearest_dist_y = 100
#     for poly in all_polys:
#         nearest_geom = nearest_points(point, poly.exterior)[1]
#         if np.abs(nearest_dist_x) > np.abs(nearest_geom.x - point.x):
#             nearest_dist_x = nearest_geom.x - point.x
#         if np.abs(nearest_dist_y) > np.abs(nearest_geom.y - point.y):
#             nearest_dist_y = nearest_geom.y - point.y
#     dx.append(nearest_geom.x - point.x)
#     dy.append(nearest_geom.y - point.y)

# print(dx, dy)

# directions = np.stack([dx, dy], axis = 1)
# norms = np.linalg.norm(directions, axis=1, keepdims=True)
# norms[norms == 0] = 1
# unit_dirs = directions / norms

# offset = .0195 + .03

# new_points = inside_points + offset * unit_dirs + directions

# # plt.scatter(new_points[:, 0], new_points[:, 1], c='g')

# N = new_points.shape[0]
# labels = np.arange(1, N+1)
# for i, (x, y) in enumerate(inside_points):
#     plt.text(x, y, str(labels[i]))

# # for i, (x, y) in enumerate(new_points):
# #     plt.text(x, y, str(labels[i]))
# for poly in all_polys:
#     exterior_line += LineString(poly.exterior.coords)
# distances = np.array([exterior_line.distance(Point(x, y)) for x, y in new_points])
# mask_far = distances >= offset
# filtered_new_points = new_points[mask_far]

# plt.scatter(filtered_new_points[:, 0], filtered_new_points[:, 1], c='r', marker='x')

# for i, (x, y) in enumerate(new_points):
#     plt.text(x, y, str(labels[i]))

# paths.show()

    