import math
import trimesh as tm
import numpy as np

# x_vertical = 0.16
# y_vertical = 0.04
# z_vertical = 0.04
# x_horizontal = 0.04
# y_horizontal = 0.16
# z_horizontal = z_vertical
# x_tot = x_vertical + x_horizontal
# y_tot = y_horizontal
# z_tot = z_vertical

filename = f'examples/sampling_c3/urdf/sphere_with_texture.obj'
mesh = tm.load_mesh(filename)
bounding_mesh_array = mesh.bounds

print(bounding_mesh_array)

x_bound = np.abs(bounding_mesh_array[0, 0]) + np.abs(bounding_mesh_array[1, 0])
y_bound = np.abs(bounding_mesh_array[0, 1]) + np.abs(bounding_mesh_array[1, 1])
# z_bound = bounding_mesh_array[0, 2]
z_bound = 0

print(x_bound, y_bound, z_bound)

urdf = f"""<?xml version="1.0"?>
<robot name="cube" xmlns:drake="https://drake.mit.edu/">
    <link name="body">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="0.37" />
            <inertia
                ixx="0.00081"
                ixy="0"
                ixz="0"
                iyy="0.00081"
                iyz="0"
                izz="0.00081"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <mesh filename="{filename}"/>
            </geometry>
            <material name="pennred">
                <color rgba="0.6 0 0 1.0" />
            </material>
        </visual>
        <collision>
            <geometry>
                <mesh filename="{filename}"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <drake:proximity_properties>
                <drake:mu_static value="0.15" />
            </drake:proximity_properties>
        </collision>
    </link>
"""

corner_positions = [
    (x_bound, 0, z_bound),
    (-x_bound, -y_bound, z_bound),
    (-x_bound, y_bound, z_bound)
]

parent_links = [
    "horizontal_link",
    "horizontal_link",
    "vertical_link"
]

for i, (pos, parent) in enumerate(zip(corner_positions,parent_links)):
    corner_name = f"corner_{i}"
    urdf += f"""
    <link name="{corner_name}">
        <inertial>
        <origin rpy="0 0 0" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
        <mass value="0.00001"/>
        <inertia
            ixx="0"
            ixy="0"
            ixz="0"
            iyy="0"
            iyz="0"
            izz="0"/>
        </inertial>
        <visual>
        <origin rpy="0 0 0" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
        <geometry>
            <sphere radius="0.01"/>
        </geometry>
        </visual>
        <collision name="sphere">
            <origin rpy="0 0 0" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
            <geometry>
                <sphere radius="0.01"/>
            </geometry>
                <drake:proximity_properties>
                <drake:mu_static value="1"/>
                <drake:mu_dynamic value="1"/>
            </drake:proximity_properties>
        </collision>
        <contact>
                    <lateral_friction value="0.0"/>
                    <rolling_friction value="0.0"/>
                    <contact_cfm value="0.0"/>
                    <contact_erp value="0.0"/>
        </contact>
    </link>
"""
    
urdf += "\n</robot>\n"

with open("T_vertical_parametric.urdf", "w") as f:
    f.write(urdf)


