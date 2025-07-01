import pymeshlab
import xml.etree.ElementTree as ET
from lxml import etree

tree = etree.parse('examples/sampling_c3/urdf/controller_push_t_white.sdf')
root = tree.getroot()

ns = {'drake': 'https://drake.mit.edu/schemas/robot'}

for mesh in root.xpath('.//mesh/uri', namespaces=ns):
    print(mesh.text)

meshes = []
for mesh in root.findall('.//mesh/uri'):
    meshes.append(mesh.text)

print("Mesh URIs found in SDF:", meshes)

# Load the mesh
ms = pymeshlab.MeshSet()
ms.load_new_mesh('path/to/your_mesh.obj')

# Get mesh info
mesh = ms.current_mesh()

print("Number of vertices:", mesh.vertex_number())
print("Number of faces:", mesh.face_number())

# Check if mesh is manifold
print("Is manifold:", mesh.is_manifold())

# Check if mesh has boundaries (holes)
print("Has boundaries:", mesh.has_boundary())

# Check if mesh is watertight (manifold + no boundaries)
is_watertight = mesh.is_manifold() and not mesh.has_boundary()
print("Is watertight:", is_watertight)

# Compute convex hull (optional)
ms.compute_convex_hull()
convex_hull_mesh = ms.current_mesh()

print("Convex hull vertices:", convex_hull_mesh.vertex_number())
print("Convex hull faces:", convex_hull_mesh.face_number())