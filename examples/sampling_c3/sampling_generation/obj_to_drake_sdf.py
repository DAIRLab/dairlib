"""Convex decomposition for sim since Drake cannot handle collisions with 
non-convex meshes without using the convex hull in order to avoid mismatched 
geometry."""

import os
import trimesh
from lxml import etree as ET

def main(obj_file, output_dir=None, model_name=None, density=1000.0,
         resolution=100000, max_hulls=10, j=0, use_quats: bool = True):
    # Normalize input paths
    obj_file = os.path.abspath(obj_file)
    if output_dir is None:
        output_dir = os.path.dirname(obj_file)
    else:
        output_dir = os.path.abspath(output_dir)
        os.makedirs(output_dir, exist_ok=True)

    obj_base = os.path.basename(obj_file)
    obj_name = os.path.splitext(obj_base)[0]
    if model_name is None:
        model_name = obj_name

    # Load mesh
    mesh = trimesh.load(obj_file, force="mesh", skip_materials=True)
    mesh.density = density
    mass = mesh.mass 
    inertia = mesh.moment_inertia

    # Perform convex decomposition
    vhacd_kwargs = {
        'maxConvexHulls': max_hulls,
        'resolution': resolution
    }
    convex_dicts = trimesh.decomposition.convex_decomposition(mesh, **vhacd_kwargs)
    convex_meshes = [trimesh.Trimesh(**d) for d in convex_dicts]

    # Export convex hulls (and original file) as separate OBJs
    mesh.vertex_normals  # include this call to ensure vn lines in obj
    mesh.export(obj_file)
    convex_paths = []
    for i, m in enumerate(convex_meshes):
        out_path = os.path.join(output_dir, f"{obj_name}_convex_{i}.obj")
        m.vertex_normals  # include this call to ensure vn lines in obj
        m.export(out_path)
        convex_paths.append(out_path)

    # Register drake namespace
    drake_ns = "drake"
    drake_uri = "uri:drake"
    ET.register_namespace(drake_ns, drake_uri)

    # Write SDF
    sdf_path = os.path.join(output_dir, obj_name + ".sdf")
    sdf = ET.Element('sdf', version="1.7")
    model = ET.SubElement(sdf, 'model', name=model_name)
    link = ET.SubElement(model, 'link', name=obj_name)

    # Inertia 
    ratio = 0.05 / mass

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "pose").text = "0 0 0 0 0 0"
    ET.SubElement(inertial, "mass").text = f"{ratio * mass:.6g}"
    inertia_elem = ET.SubElement(inertial, "inertia")
    ET.SubElement(inertia_elem, "ixx").text = f"{ratio * inertia[0,0]:.8g}"
    ET.SubElement(inertia_elem, "iyy").text = f"{ratio * inertia[1,1]:.8g}"
    ET.SubElement(inertia_elem, "izz").text = f"{ratio * inertia[2,2]:.8g}"
    ET.SubElement(inertia_elem, "ixy").text = f"{ratio * inertia[0,1]:.8g}"
    ET.SubElement(inertia_elem, "ixz").text = f"{ratio * inertia[0,2]:.8g}"
    ET.SubElement(inertia_elem, "iyz").text = f"{ratio * inertia[1,2]:.8g}"

    # For each convex hull, add visual and collision
    for i, convex_path in enumerate(convex_paths):
        convex_obj = os.path.basename(convex_path)

        color = str((50 * (j + 1)) % 256 / 255.0) + " " + str((100 * (j + 1)) % 256 / 255.0) + " " + str((150 * (j + 1)) % 256 / 255.0) + " 1" 

        # Visual
        visual = ET.SubElement(link, "visual", name=f"convex_{i}")
        ET.SubElement(visual, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        mesh_elem = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh_elem, "uri").text = convex_obj
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "diffuse").text = color

        # Collision
        collision = ET.SubElement(link, "collision", name=f"convex_{i}_volume")
        ET.SubElement(collision, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        mesh_elem = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh_elem, "uri").text = convex_obj

        # Drake proximity properties
        prox = ET.SubElement(
            collision,
            ET.QName(drake_uri, "proximity_properties")
        )
        ET.SubElement(prox, ET.QName(drake_uri, "compliant_hydroelastic"))
        ET.SubElement(prox, ET.QName(drake_uri, "hydroelastic_modulus")).text = " 3.0e7 "
        ET.SubElement(prox, ET.QName(drake_uri, "mesh_resolution_hint")).text = " 0.02 "
        ET.SubElement(prox, ET.QName(drake_uri, "hunt_crossley_dissipation")).text = "10"
        ET.SubElement(prox, ET.QName(drake_uri, "mu_dynamic")).text = "0.3"

    # For planar rotations, add extra constraints.
    if not use_quats:
        base_link = ET.SubElement(model, 'link', name=f'{obj_name}_base')
        fake_x = ET.SubElement(model, 'link', name='fake_x')
        fake_y = ET.SubElement(model, 'link', name='fake_y')
        fake_z = ET.SubElement(model, 'link', name='fake_z')

        base_to_x = ET.SubElement(model, 'joint', name='base_to_x', type='prismatic')
        ET.SubElement(base_to_x, 'parent').text = f'{obj_name}_base'
        ET.SubElement(base_to_x, 'child').text = 'fake_x'
        base_to_x_axis = ET.SubElement(base_to_x, 'axis')
        ET.SubElement(base_to_x_axis, 'xyz').text = '1 0 0'
        base_to_x_limit = ET.SubElement(base_to_x_axis, 'limit')
        ET.SubElement(base_to_x_limit, 'lower').text = '-1.79769e+308'
        ET.SubElement(base_to_x_limit, 'upper').text = '1.79769e+308'

        x_to_y = ET.SubElement(model, 'joint', name='x_to_y', type='prismatic')
        ET.SubElement(x_to_y, 'parent').text = 'fake_x'
        ET.SubElement(x_to_y, 'child').text = 'fake_y'
        x_to_y_axis = ET.SubElement(x_to_y, 'axis')
        ET.SubElement(x_to_y_axis, 'xyz').text = '0 1 0'
        x_to_y_limit = ET.SubElement(x_to_y_axis, 'limit')
        ET.SubElement(x_to_y_limit, 'lower').text = '-1.79769e+308'
        ET.SubElement(x_to_y_limit, 'upper').text = '1.79769e+308'

        y_to_z = ET.SubElement(model, 'joint', name='y_to_z', type='prismatic')
        ET.SubElement(y_to_z, 'parent').text = 'fake_y'
        ET.SubElement(y_to_z, 'child').text = 'fake_z'
        y_to_z_axis = ET.SubElement(y_to_z, 'axis')
        ET.SubElement(y_to_z_axis, 'xyz').text = '0 0 1'
        y_to_z_limit = ET.SubElement(y_to_z_axis, 'limit')
        ET.SubElement(y_to_z_limit, 'lower').text = '-1.79769e+308'
        ET.SubElement(y_to_z_limit, 'upper').text = '1.79769e+308'

        z_to_rot = ET.SubElement(model, 'joint', name='z_to_rot', type='continuous')
        ET.SubElement(z_to_rot, 'parent').text = 'fake_z'
        ET.SubElement(z_to_rot, 'child').text = obj_name
        z_to_rot_axis = ET.SubElement(z_to_rot, 'axis')
        ET.SubElement(z_to_rot_axis, 'xyz').text = '0 0 1'

    # Pretty print and save
    comment = ET.Comment(" This SDF was autogenerated by obj_to_drake_sdf.py ")
    sdf.addprevious(comment)
    tree = ET.ElementTree(sdf)
    tree.write(sdf_path, pretty_print=True, xml_declaration=True, encoding="utf-8")
    print(f"Wrote SDF to {sdf_path}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python obj_to_drake_sdf.py path/to/model.obj [output_dir]")
        exit(1)
    obj_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else None
    main(obj_path, out_dir)
