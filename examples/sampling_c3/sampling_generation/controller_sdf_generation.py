import sys
import numpy as np
import os

SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="t_shape">    
    <link name="vertical_link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>1</mass>
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

      <visual name="vertical_link">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{OBJ}</uri>
          </mesh>
        </geometry>
        <material>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="vertical_link_volume">
        <pose>0 0 0 0 0 0</pose>
            <geometry>
          <mesh>
            <uri>{OBJ}</uri>
          </mesh>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus> 3.0e7 </drake:hydroelastic_modulus>
          <drake:mesh_resolution_hint> 0.18 </drake:mesh_resolution_hint>
          <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation>
          <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
      </collision>
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
  </model>
</sdf>
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

def make_sdf(obj_filename, output_path=None):
    c_nxynz, c_nxnynz, c_xynz = get_obj_corners(obj_filename)
    obj_basename = os.path.basename(obj_filename)
    sdf_xml = SDF_TEMPLATE.format(
        OBJ=obj_basename,
        CORNER_NXYNZ=c_nxynz,
        CORNER_NXNYNZ=c_nxnynz,
        CORNER_XYNZ=c_xynz,
    )
    if output_path:
        with open(output_path, 'w') as f:
            f.write(sdf_xml)
        print(f"Wrote SDF to {output_path}")
    else:
        print(sdf_xml)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python controller_sdf_generation.py <obj_file> [output_file.sdf]")
        sys.exit(1)
    obj_file = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    make_sdf(obj_file, out_path)
