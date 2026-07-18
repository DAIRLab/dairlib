import trimesh
import os

def center_mesh_at_com(input_path, output_path=None):
    # Load the mesh
    mesh = trimesh.load(input_path, force='mesh')

    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError("Loaded object is not a single mesh.")

    # Compute the center of mass
    com = mesh.center_mass

    # Translate the mesh so that COM is at the origin
    mesh.apply_translation(-com)

    # Determine output path
    if output_path is None:
        base, ext = os.path.splitext(input_path)
        output_path = base + "_centered.obj"

    # Export the translated mesh
    mesh.export(output_path)
    print(f"Mesh centered at COM and saved to: {output_path}")
    return output_path

names = ['D', 'A', 'I', 'R', 'C', '3', 'plus']
urdf_path = "examples/sampling_c3/urdf"

for name in names:
    file = name + ".obj"
    path = os.path.join(urdf_path, file)

    center_mesh_at_com(path)