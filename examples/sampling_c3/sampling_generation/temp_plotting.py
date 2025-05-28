import numpy as np
import matplotlib.pyplot as plt
import trimesh as tm

original = np.loadtxt('examples/sampling_c3/sampling_generation/unioned_points.csv', delimiter=',')
points = np.loadtxt('examples/sampling_c3/sampling_generation/sampled_points.csv', delimiter=',')
ring = np.loadtxt('examples/sampling_c3/sampling_generation/buffered_ring.csv', delimiter=',')
hull = np.loadtxt('examples/sampling_c3/sampling_generation/convex_hull.csv', delimiter=',')
plt.scatter(original[:, 0], original[:, 1], color='red', label='Unioned Graph', s=5)
plt.scatter(ring[:, 0], ring[:, 1], color='orange', label='Buffered Ring', s=5)
plt.scatter(points[:, 0], points[:, 1], color='blue', label='Sampled Points', s=25)
# plt.scatter(hull[:, 0], hull[:, 1], color='purple', label='Convex Hull', s=5)
plt.axis('equal')

plt.legend()

mesh = tm.load_mesh('examples/sampling_c3/urdf/toblerone_bsdf_mesh.obj')
print(mesh.is_watertight)

print("Vertices:", mesh.vertices.shape[0])
print("Edges:", mesh.edges.shape[0])
print("Faces:", mesh.faces.shape[0])

vertices = mesh.vertices
edges = mesh.edges
intersections = []

for edge in edges:
    v1 = vertices[edge[0]]
    v2 = vertices[edge[1]]
    z1, z2 = v1[2], v2[2]
    if (z1 - 0.005) * (z2 - 0.005) < 0:
        t = (0.005 - z1) / (z2 - z1)
        intersection = v1 + t * (v2 - v1)
        intersections.append(intersection)

intersections = np.array(intersections)
print(f'Number of intersections at z=0.005: {len(intersections)}')
plt.scatter(intersections[:, 0], intersections[:, 1], color='green', label='Mesh Vertices at z=0.005', s=5)

# Save the mesh as an OBJ file
mesh.export('examples/sampling_c3/sampling_generation/python.obj', file_type='obj')

plt.show()

