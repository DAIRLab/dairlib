import numpy as np
import matplotlib.pyplot as plt
import trimesh as tm

#original = np.loadtxt('examples/sampling_c3/sampling_generation/unioned_points.csv', delimiter=',')
#points = np.loadtxt('examples/sampling_c3/sampling_generation/new_sampled_points.csv', delimiter=',')
#ring = np.loadtxt('examples/sampling_c3/sampling_generation/new_ring.csv', delimiter=',')
# plt.scatter(original[:, 0], original[:, 1], color='red', label='Unioned Graph', s=5)
#plt.scatter(ring[:, 0], ring[:, 1], color='orange', label='Buffered Ring', s=5)
#plt.scatter(points[:, 0], points[:, 1], color='blue', label='Sampled Points', s=5)
#plt.axis('equal')

plt.legend()

mesh = tm.load_mesh('examples/sampling_c3/urdf/push_t_white.obj')
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
    if (z1 - 0.01) * (z2 - 0.01) < 0:
        t = (0.01 - z1) / (z2 - z1)
        intersection = v1 + t * (v2 - v1)
        intersections.append(intersection)

intersections = np.array(intersections)
print(f'Number of intersections at z=0.01: {len(intersections)}')



density = 1240
mass = density * mesh.volume
inertia = mesh.moment_inertia

com = mesh.center_mass
T = np.eye(4)
T[:3, 3] = com

# Call transform_inertia with the inertia tensor, mass, and transform matrix T
inertia_at_com = tm.inertia.transform_inertia(
    transform=T,  # translation vector
    inertia_tensor=inertia,
    parallel_axis=True,          # apply parallel axis theorem
    mass=mass                   # mass of the object
)
print("Mass:", mass)
print("COM:", mesh.center_mass)
print("Inertia at COM:", com)




plt.scatter(intersections[:, 0], intersections[:, 1], color='green', label='Mesh Vertices at z=0.01', s=5)

# Save the mesh as an OBJ file
mesh.export('examples/sampling_c3/sampling_generation/python.obj', file_type='obj')

plt.show()

vertices_xy = mesh.vertices[:, :2]

# Get COM XY
com_xy = mesh.center_mass[:2]

plt.figure(figsize=(8,8))

# Plot mesh vertices as dots (top-down view, XY plane)
plt.scatter(vertices_xy[:, 0], vertices_xy[:, 1], s=1, alpha=0.3, label='Mesh vertices')

# Plot COM as a red star
plt.scatter(com_xy[0], com_xy[1], color='red', s=100, marker='*', label='Center of Mass')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Top-Down View (XY Plane) of Mesh and COM')
plt.axis('equal')  # Equal aspect ratio for x and y
plt.legend()
plt.grid(True)
plt.show()

