import trimesh
import numpy as np
from lxml import etree
from collections import defaultdict

def load_mesh_from_sdf(sdf_path):
    """
    Parse the SDF file, extract the mesh file paths from <mesh><uri> elements,
    then load the first mesh found using trimesh.
    """
    tree = etree.parse(sdf_path)
    root = tree.getroot()

    # You might need to adjust this namespace depending on your sdf
    ns = {'sdf': 'http://sdformat.org/schemas/root.xsd'}

    # Find all mesh URIs regardless of namespace (just fallback to ignore ns)
    mesh_uris = root.xpath('.//mesh/uri')

    if not mesh_uris:
        raise RuntimeError("No <mesh><uri> elements found in SDF file.")

    # The mesh URI might be a relative path or have "model://" prefix
    mesh_path = mesh_uris[0].text
    if mesh_path.startswith("model://"):
        # Replace model:// with local model path prefix
        # Adjust this base path accordingly
        base_model_path = "/path/to/your/models/folder"
        mesh_path = mesh_path.replace("model://", base_model_path + "/")
    print(f"Loading mesh from SDF URI: {mesh_path}")

    return trimesh.load(mesh_path, force='mesh')


def find_non_manifold_edges(mesh: trimesh.Trimesh):
    from collections import defaultdict

    edge_face_count = defaultdict(int)

    for face in mesh.faces:
        face_edges = [
            tuple(sorted((face[0], face[1]))),
            tuple(sorted((face[1], face[2]))),
            tuple(sorted((face[2], face[0]))),
        ]
        for e in face_edges:
            edge_face_count[e] += 1

    non_manifold_edges = [e for e, count in edge_face_count.items() if count > 2]
    return non_manifold_edges


def visualize_non_manifold_edges(mesh: trimesh.Trimesh, non_manifold_edges):
    if len(non_manifold_edges) == 0:
        print("No non-manifold edges to visualize.")
        mesh.show()
        return

    vertex_index_map = {}
    unique_vertices_list = []
    edges_indices = []

    # Build unique vertex list and map old vertex indices to new indices
    for edge in non_manifold_edges:
        for v in edge:
            if v not in vertex_index_map:
                vertex_index_map[v] = len(unique_vertices_list)
                unique_vertices_list.append(mesh.vertices[v])

    unique_vertices = np.array(unique_vertices_list)

    # Remap edges to new indices
    for edge in non_manifold_edges:
        i0 = vertex_index_map[edge[0]]
        i1 = vertex_index_map[edge[1]]
        if i0 != i1:  # skip zero-length edges
            edges_indices.append([i0, i1])

    edges_indices = np.array(edges_indices)
    if edges_indices.size == 0:
        print("No valid non-manifold edges to visualize after filtering zero-length edges.")
        return

    # Remove duplicate vertices using trimesh's utility function (optional)
    unique_vertices, inverse_indices = trimesh.points.group_rows(unique_vertices, decimals=8, require_count=1)

    # Remap edges_indices according to inverse_indices to match deduplication
    edges_indices = np.array([
        [inverse_indices[i], inverse_indices[j]] for i, j in edges_indices
    ])

    # Remove any zero-length edges after deduplication
    edges_indices = np.array([e for e in edges_indices if e[0] != e[1]])
    if len(edges_indices) == 0:
        print("No valid edges to show after deduplication.")
        mesh.show()
        return

    # Create the Path3D object
    edge_paths = trimesh.path.Path3D(vertices=unique_vertices, edges=edges_indices)
    edge_paths.colors = [255, 0, 0, 255]  # red color

    # Show scene with mesh and highlighted edges
    scene = trimesh.Scene([mesh, edge_paths])
    scene.show()

def find_zero_length_edges(mesh, edges):

    zero_length_edges = []

    for edge in edges:
        v0, v1 = mesh.vertices[edge[0]], mesh.vertices[edge[1]]
        if np.allclose(v0, v1):
            zero_length_edges.append(edge)

    return zero_length_edges


def faces_with_repeated_vertices(mesh: trimesh.Trimesh):
    """
    Return a list of indices of faces that have repeated vertices.
    
    A face has repeated vertices if any two or more vertex indices are the same.
    """
    repeated_faces = []
    for i, face in enumerate(mesh.faces):
        # If any two indices are equal, the face is degenerate
        if len(set(face)) < 3:
            repeated_faces.append(i)
    return repeated_faces

def analyze_mesh(mesh: trimesh.Trimesh, name="mesh"):
    print(f"--- Analyzing {name} ---")
    print(f"Vertices: {len(mesh.vertices)}")
    print(f"Faces: {len(mesh.faces)}")
    print(f"Is watertight: {mesh.is_watertight}")
    print(f"Is manifold: {mesh.is_watertight and mesh.is_winding_consistent}")
    print(f"Has consistent winding: {mesh.is_winding_consistent}")
    print(f"Euler number: {mesh.euler_number}")

    # Check normals orientation
    if mesh.vertex_normals is None or len(mesh.vertex_normals) == 0:
        mesh.rezero()  # attempt to fix
        mesh.compute_vertex_normals()
    flipped_faces = mesh.faces[~mesh.is_winding_consistent]
    print(f"Number of faces with inconsistent winding: {len(flipped_faces)}")

    # Bounding box info
    bbox = mesh.bounds
    print(f"Bounding box min: {bbox[0]}")
    print(f"Bounding box max: {bbox[1]}")
    print(f"Bounding box size: {bbox[1] - bbox[0]}")

    print()

    non_manifold_edges = find_non_manifold_edges(mesh)
    visualize_non_manifold_edges(mesh, non_manifold_edges)

    bad_faces = faces_with_repeated_vertices(mesh)
    print(f"Found {len(bad_faces)} faces with repeated vertices.")
    if bad_faces:
        print("Indices of degenerate faces:", bad_faces)


def main():
    push_t_white_path = "examples/sampling_c3/urdf/push_t_white.obj"
    bundlesdf_path = "examples/sampling_c3/script_output/repaired_controller.sdf"

    push_mesh = trimesh.load(push_t_white_path, force='mesh')
    bundle_mesh = load_mesh_from_sdf(bundlesdf_path)

    #analyze_mesh(push_mesh, "push_t_white.obj")
    analyze_mesh(bundle_mesh, "bundle.sdf mesh")

if __name__ == "__main__":
    main()
