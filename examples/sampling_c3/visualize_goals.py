"""Help manually determine goal poses for spelling things."""

import os.path as op
import sys
import tempfile
import yaml
from pydrake.visualization import ModelVisualizer

from multiyaml_rewrite import main as multiyaml_rewrite


DAIRLIB_DIR = op.abspath(op.dirname(op.dirname(op.dirname(__file__))))

URDF_TEMPLATE = f"""
<robot name="multi-object">
</robot>
"""
LINK_TEMPLATE = """
    <link name="{OBJECT_NAME}">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </inertial>
        <visual>
            <geometry>
            <mesh filename="{OBJ_FILEPATH}"/>
            </geometry>
            <material>
            <color rgba="{RED} {GREEN} {BLUE} 1.0" />
            </material>
        </visual>
    </link>
"""

# Before anything, run the multiyaml rewrite.
multiyaml_rewrite()


# First, load the object models from the current object settings.
yaml_path = op.join(
    DAIRLIB_DIR,
    'examples/sampling_c3/anything/parameters',
    'sampling_c3_controller_params.yaml')
with open(yaml_path, 'r') as f:
    settings = yaml.safe_load(f)
object_models = settings['object_models']

# Write one temporary URDF file containing floating bodies for all objects.
contents = URDF_TEMPLATE
for i, obj in enumerate(object_models):
    obj_name = op.splitext(op.basename(obj))[0].replace(
        '_controller', '').replace('_shape', '')
    if object_models.count(obj) > 1:
        obj_name += f'_{i}'
    r = (50 * (i + 1)) % 256 / 255.0
    g = (100 * (i + 1)) % 256 / 255.0
    b = (150 * (i + 1)) % 256 / 255.0
    full_obj_path = op.join(DAIRLIB_DIR, obj.replace('_controller.sdf', '.obj'))
    contents = contents.replace(
        '</robot>',
        LINK_TEMPLATE.format(
            OBJECT_NAME=obj_name,
            OBJ_FILEPATH=full_obj_path,
            RED=r,
            GREEN=g,
            BLUE=b,
        ) + '\n</robot>'
    )

# Write the string to a tmp file.
with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as tmp_file:
    tmp_file.write(contents.encode())
    urdf_path = tmp_file.name

# Get the starting goal positions.
goal_yaml_path = op.join(DAIRLIB_DIR, settings['goal_params_file'])
with open(goal_yaml_path, 'r') as f:
    goal_settings = yaml.safe_load(f)
xyzs = goal_settings['fixed_target_positions']
quats = goal_settings['fixed_target_orientations']
position = []
for i in range(len(object_models)):
    position += quats[i] + xyzs[i]

# Open the Drake model visualizer.  The bash command is:
"""python -m pydrake.visualization.model_visualizer --open-window
{urdf_path}"""
visualizer = ModelVisualizer()  #browser_new=True)
visualizer.AddModels(urdf_path)
visualizer.Run(position=position)
