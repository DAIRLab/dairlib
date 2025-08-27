import sys
import numpy as np
import os
from lxml import etree as ET

SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="t_shape">    
    <link name="{OBJ}">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>1.0</mass>
        <!-- Inertia values were calculated to be consistent with the mass and
             geometry size, assuming a uniform density. -->
        <inertia>
            <ixx>0.003</ixx>
            <iyy>0.003 </iyy>
            <izz>0.006 </izz>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyz>0.0</iyz>
        </inertia>
      </inertial>
      {CONVEX_PARTS}
      <collision name="corner_nxynz_">
        <geometry>
          <sphere>
            <radius>0.001</radius>
          </sphere>
        </geometry>
        <drake:proximity_properties>
        <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
        <pose> {CORNER_NXYNZ} 0 0 0</pose>
      </collision>   
      <collision name="corner_nxnynz">
        <geometry>
          <sphere>
            <radius>0.001</radius>
          </sphere>
        </geometry>
        <drake:proximity_properties>
        <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
        <pose> {CORNER_NXNYNZ} 0 0 0</pose>
      </collision>
      <collision name="corner_xynz">
        <geometry>
          <sphere>
            <radius>0.001</radius>
          </sphere>
        </geometry>
        <drake:proximity_properties>
        <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
        <pose> {CORNER_XYNZ} 0 0 0</pose>
      </collision>
    </link>
    {EXTRAS}
  </model>
</sdf>
"""

PLANAR_ROTATION_EXTRAS = """
    <link name="{OBJ}_base" />
    <link name="fake_x" />
    <link name="fake_y" />
    <link name="fake_z" />

    <joint name="base_to_x" type="prismatic">
      <parent> {OBJ}_base </parent>
      <child> fake_x </child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
            <effort>0</effort>   <!-- Mark as unactuated -->
        </limit>
      </axis>
    </joint>

    <joint name="x_to_y" type="prismatic">
      <parent> fake_x </parent>
      <child> fake_y </child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
            <effort>0</effort>   <!-- Mark as unactuated -->
        </limit>
      </axis>
    </joint>

    <joint name="y_to_z" type="prismatic">
      <parent> fake_y </parent>
      <child> fake_z </child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
            <effort>0</effort>   <!-- Mark as unactuated -->
        </limit>
      </axis>
    </joint>

    <joint name="z_to_rot" type="continuous">
      <parent> fake_z </parent>
      <child> {OBJ} </child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
            <effort>0</effort>   <!-- Mark as unactuated -->
        </limit>
      </axis>
    </joint>
"""

def get_obj_corners(obj_file):
    # Read all vertices
    vertices = []
    with open(obj_file, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                if len(parts) != 4:
                    continue
                x, y, z = map(float, parts[1:])
                vertices.append([x, y, z])
    if not vertices:
        raise RuntimeError(f"No vertices found in {obj_file}")
    arr = np.array(vertices)
    min_x = np.min(arr[:,0])
    max_x = np.max(arr[:,0])
    min_y = np.min(arr[:,1])
    max_y = np.max(arr[:,1])
    min_z = np.min(arr[:,2])
    max_z = np.max(arr[:,2])
    mid_y = (min_y + max_y) / 2.0
    # corner_nxynz: min x, max y, min z
    c_nxynz = f"{min_x:.8f} {max_y:.8f} {min_z:.8f}"
    # corner_nxnynz: min x, min y, min z
    c_nxnynz = f"{min_x:.8f} {min_y:.8f} {min_z:.8f}"
    # corner_xynz: max x, mid y, min z
    c_xynz = f"{max_x:.8f} {mid_y:.8f} {min_z:.8f}"
    return c_nxynz, c_nxnynz, c_xynz

def make_convex_parts(convex_paths, base_name):
    drake_uri = "uri:drake"
    ET.register_namespace("drake", drake_uri)

    link = ET.Element('link', name=base_name)

    for i, convex_path in enumerate(convex_paths):
        convex_obj = os.path.basename(convex_path)

        visual = ET.SubElement(link, "visual", name=f"convex_{i}")
        ET.SubElement(visual, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        mesh_elem = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh_elem, "uri").text = convex_obj
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "diffuse").text = "0 0 0 1"

        collision = ET.SubElement(link, "collision", name=f"convex_{i}_volume")
        ET.SubElement(collision, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        mesh_elem = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh_elem, "uri").text = convex_obj

        prox = ET.SubElement(
            collision,
            ET.QName(drake_uri, "proximity_properties")
        )
        ET.SubElement(prox, ET.QName(drake_uri, "compliant_hydroelastic"))
        ET.SubElement(prox, ET.QName(drake_uri, "hydroelastic_modulus")).text = "3.0e7"
        ET.SubElement(prox, ET.QName(drake_uri, "mesh_resolution_hint")).text = "0.18"
        ET.SubElement(prox, ET.QName(drake_uri, "hunt_crossley_dissipation")).text = "10"
        ET.SubElement(prox, ET.QName(drake_uri, "mu_dynamic")).text = "0.3"

    # Serialize only the children of <link>, joining them into one string
    parts = []
    for child in link:
        parts.append(ET.tostring(child, pretty_print=True, encoding="unicode"))

    return ''.join(parts)

def make_sdf(obj_filename, output_path=None, use_quats: bool = True):
    c_nxynz, c_nxnynz, c_xynz = get_obj_corners(obj_filename)
    obj_basename = os.path.basename(obj_filename)
    base_name = os.path.splitext(obj_basename)[0]

    convex_files = [f"{base_name}_convex_{i}.obj" for i in range(10)]
    convex_parts = make_convex_parts(convex_files, base_name)

    sdf_template = SDF_TEMPLATE
    extras = '' if use_quats else PLANAR_ROTATION_EXTRAS.format(OBJ=base_name)

    sdf_xml = sdf_template.format(
        OBJ = base_name,
        CONVEX_PARTS = convex_parts,
        CORNER_NXYNZ=c_nxynz,
        CORNER_NXNYNZ=c_nxnynz,
        CORNER_XYNZ=c_xynz,
        EXTRAS=extras,
    )
    if output_path:
        with open(output_path, 'w') as f:
            f.write(sdf_xml)
        print(f"Wrote SDF to {output_path}")
    else:
        print(sdf_xml)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python controller_sdf_generation.py <obj_file> " + \
              "[output_file.sdf --use-quats]")
        sys.exit(1)
    obj_file = sys.argv[1]
    out_path = sys.argv[2] if (len(sys.argv) > 2) and \
      (sys.argv[2] != "--use-quats") else None
    use_quats = "--use-quats" in sys.argv
    make_sdf(obj_file, out_path, use_quats=use_quats)
