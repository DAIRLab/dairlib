import trimesh
import numpy as np
from lxml import etree

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

def main():
    push_t_white_path = "examples/sampling_c3/urdf/push_t_white.obj"
    bundlesdf_path = "examples/sampling_c3/script_output/your_mesh_centered_controller.sdf"

    push_mesh = trimesh.load(push_t_white_path, force='mesh')
    bundle_mesh = load_mesh_from_sdf(bundlesdf_path)

    analyze_mesh(push_mesh, "push_t_white.obj")
    analyze_mesh(bundle_mesh, "bundle.sdf mesh")

if __name__ == "__main__":
    main()
