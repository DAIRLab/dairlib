import sys, os, re, trimesh
from ruamel.yaml import YAML
from sampling_generation.controller_sdf_generation import make_sdf
from sampling_generation.obj_to_drake_sdf import main as obj_to_drake_sdf
from sampling_generation.obj_z_planes import get_min_z_from_obj, get_max_z_from_obj

yaml_io = YAML()
yaml_io.preserve_quotes = True
yaml_io.indent(sequence=4, offset=2)
yaml_io.width = 1000
yaml_io.default_flow_style = True

def coarsify_obj(path):
    mesh = trimesh.load(path)
    num_faces = len(mesh.faces)
    if num_faces > 150000:
        ratio = 1 - (150000/num_faces)
        simplified = mesh.simplify_quadric_decimation(ratio)
        simplified.export(path)
        return True
    return False

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml_io.load(f)

def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml_io.dump(data, f)

def get_num_objects_from_yaml(yaml_path: str) -> int:
    with open(yaml_path, 'r') as f:
        data = yaml_io.load(f)
    models = data.get("base_names")
    return len(models)

def calculate_contacts(num_objects: int, include_walls: int) -> int:

    return int(num_objects * (num_objects - 1) // 2 + num_objects * 3 + 1 + include_walls)

def choose_2(num_objects: int) -> int:
    return int(num_objects * (num_objects - 1) // 2)


def make_walls(samp_c3_options_yaml_path):
    samp_c3_options = load_yaml(samp_c3_options_yaml_path)

    workspace_limits = samp_c3_options['workspace_limits']
    length = workspace_limits[0][4]
    width = workspace_limits[1][4] - workspace_limits[1][3]

    side_size_str = str(length) + " 0.1 0.2"

    left_wall = f"""<?xml version="1.0" encoding="utf-8"?>
        <robot name="left_wall">
        <link name="left_wall">
            <inertial>
            <mass value="1"/>
            <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
            <visual>
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{side_size_str}"/>
            </geometry>
            </visual>
            <collision name="left_wall">
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{side_size_str}"/>
            </geometry>
            <drake:proximity_properties>
                <drake:mu_static value="0.5"/>
                <drake:mu_dynamic value="0.5"/>
            </drake:proximity_properties>
            </collision>
            <contact>
            <lateral_friction value="0.0"/>
            <rolling_friction value="0.0"/>
            <contact_cfm value="0.0"/>
            <contact_erp value="0.0"/>
            </contact>
        </link>

        <drake:collision_filter_group name="left_wall_group">
            <drake:member link="left_wall"/>
            <drake:ignored_collision_filter_group name="left_wall_group"/>
        </drake:collision_filter_group>
        </robot>
        """
    
    right_wall = f"""<?xml version="1.0" encoding="utf-8"?>
        <robot name="right_wall">
        <link name="right_wall">
            <inertial>
            <mass value="1"/>
            <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
            <visual>
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{side_size_str}"/>
            </geometry>
            </visual>
            <collision name="right_wall">
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{side_size_str}"/>
            </geometry>
            <drake:proximity_properties>
                <drake:mu_static value="0.5"/>
                <drake:mu_dynamic value="0.5"/>
            </drake:proximity_properties>
            </collision>
            <contact>
            <lateral_friction value="0.0"/>
            <rolling_friction value="0.0"/>
            <contact_cfm value="0.0"/>
            <contact_erp value="0.0"/>
            </contact>
        </link>

        <drake:collision_filter_group name="right_wall_group">
            <drake:member link="right_wall"/>
            <drake:ignored_collision_filter_group name="right_wall_group"/>
        </drake:collision_filter_group>
        </robot>
        """
    
    front_size_str = "0.1 " + str(width) + " 0.2"

    front_wall = f"""<?xml version="1.0" encoding="utf-8"?>
        <robot name="front_wall">
        <link name="front_wall">
            <inertial>
            <mass value="1"/>
            <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
            <visual>
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{front_size_str}"/>
            </geometry>
            </visual>
            <collision name="front_wall">
            <origin xyz="0 0 0"/>
            <geometry>
                <box size="{front_size_str}"/>
            </geometry>
            <drake:proximity_properties>
                <drake:mu_static value="0.5"/>
                <drake:mu_dynamic value="0.5"/>
            </drake:proximity_properties>
            </collision>
            <contact>
            <lateral_friction value="0.0"/>
            <rolling_friction value="0.0"/>
            <contact_cfm value="0.0"/>
            <contact_erp value="0.0"/>
            </contact>
        </link>

        <drake:collision_filter_group name="front_wall_group">
            <drake:member link="front_wall"/>
            <drake:ignored_collision_filter_group name="front_wall_group"/>
        </drake:collision_filter_group>
        </robot>
        """
    
    with open('examples/sampling_c3/urdf/wall_left.urdf', 'w') as f:
        f.write(left_wall)
    print(f"Wrote left wall")

    with open('examples/sampling_c3/urdf/wall_right.urdf', 'w') as f:
        f.write(right_wall)
    print(f"Wrote right wall")
    with open('examples/sampling_c3/urdf/wall_front.urdf', 'w') as f:
        f.write(front_wall)
    print(f"Wrote front wall")

yaml_path = "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
num_objects = get_num_objects_from_yaml(yaml_path)
num_contacts = calculate_contacts(num_objects, 0)
print("Number of objects:", num_objects)
print("Number of contacts no walls:", num_contacts)
def process_obj(
    base_name: str,
    urdf_dir: str,
    controller_yaml_path: str,
    vis_yaml_path: str,
    sim_yaml_path: str,
    goal_yaml_path: str,
    sampling_yaml_path: str,
    repos_yaml_path: str,
    samp_c3_options_yaml_path: str,
    index: int
):
    print(f"\nProcessing object: {base_name}")
    output_dir = os.path.join(urdf_dir, base_name)
    os.makedirs(output_dir, exist_ok=True)

    obj_file = os.path.join(urdf_dir, f"{base_name}.obj")
    if not os.path.isfile(obj_file):
        obj_file = os.path.join(output_dir, f"{base_name}.obj")
        
    is_coarse = coarsify_obj(obj_file)

    # Create SDF file paths
    controller_sdf_path = os.path.join(output_dir, f"{base_name}_controller.sdf")
    combined_sdf_path = os.path.join(output_dir, f"{base_name}.sdf")

    # Copy original OBJ to output directory

    os.system(f"cp {obj_file} {output_dir}/")
    print(f"Copied {obj_file} → {output_dir}/")

    obj_file = os.path.join(output_dir, f"{base_name}.obj")


    # Generate SDFs
    obj_to_drake_sdf(obj_file, output_dir, j=index)
    make_sdf(obj_file, controller_sdf_path)

    # Get min/max z-heights
    min_z = get_min_z_from_obj(obj_file)
    max_z = get_max_z_from_obj(obj_file)
    print(f"✅ {base_name} → min_z={min_z:.6f}, max_z={max_z:.6f}")
    print(f"✅ Finished processing {base_name}\n")

def set_object_paths(index, base_name, output_dir, controller_yaml, vis_yaml, sim_yaml):

    controller_sdf_path = os.path.join(output_dir, f"{base_name}_controller.sdf")
    combined_sdf_path = os.path.join(output_dir, f"{base_name}.sdf")

    print(f"controller_sdf_path: {controller_sdf_path}")
    print(f"combined_sdf_path: {combined_sdf_path}")

    controller_yaml["object_models"][index] = controller_sdf_path

    vis_yaml["object_vis_models"][index] = combined_sdf_path

    sim_yaml["object_models"][index] = combined_sdf_path



# Build q_vector for n objects
def build_q_vector(num_objects: int) -> list:
    EE_POSITION = [0.01, 0.01, 0.01]
    OBJECT_ORIENTATION = [0.1, 0.1, 0.1, 0.1]
    OBJECT_POSITION = [300, 300, 120]
    EE_LINEAR_VELOCITY = [5, 5, 5]
    OBJECT_ANGULAR_VELOCITY = [0.05, 0.05, 0.05]
    OBJECT_LINEAR_VELOCITY = [0.05, 0.05, 0.05]

    q_vector = []

    # 1. EE position
    q_vector.extend(EE_POSITION)

    # 2. Object orientations & positions
    for _ in range(num_objects):
        q_vector.extend(OBJECT_ORIENTATION)
        q_vector.extend(OBJECT_POSITION)

    # 3. EE linear velocity
    q_vector.extend(EE_LINEAR_VELOCITY)

    # 4. Object angular & linear velocities
    for _ in range(num_objects):
        q_vector.extend(OBJECT_ANGULAR_VELOCITY)
        q_vector.extend(OBJECT_LINEAR_VELOCITY)

    return q_vector

def update_c3_options(is_c3_plus, samp_c3_options_yaml_path): 
    samp_c3_options_yaml = load_yaml(samp_c3_options_yaml_path)

    include_walls = 2 if (controller_yaml['include_walls']) else 0
    samp_c3_options_yaml['resolve_contacts_to_lists'] = [[0, 1, num_objects * 3, choose_2(num_objects), include_walls * num_objects]]
    samp_c3_options_yaml["q_vector"] = build_q_vector(num_objects)
    samp_c3_options_yaml["q_vector_position"] = build_q_vector(num_objects)

    samp_c3_options_yaml["g_x"] = [950] * 3 + [1] * (7*num_objects) + [0.1] * (3 + 6*num_objects)
    samp_c3_options_yaml["g_gamma_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["g_lambda_n_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["g_lambda_t_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]

    if (is_c3_plus):
        samp_c3_options_yaml["g_eta_slack_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["g_eta_n_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["g_eta_t_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_eta_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_lambda_list"] = [[2] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
    else: 
        samp_c3_options_yaml["g_lambda_list"] = [[0.05] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]

    samp_c3_options_yaml["u_gamma_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["u_lambda_n_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["u_lambda_t_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
    
    if (is_c3_plus):
        samp_c3_options_yaml["u_eta_slack_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["u_eta_n_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["u_eta_t_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]   
        samp_c3_options_yaml["u_lambda_list"] = [[20] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_eta_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_x"] = [0] * (6 + (13 * num_objects))
    else:
        samp_c3_options_yaml["u_lambda_list"] = [[10] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_x"] = [10] * 3 + [100, 100, 100, 100, 10, 10, 10] * num_objects + [8] * 3 + [1] * (6*num_objects)

    samp_c3_options_yaml["g_gamma_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["g_lambda_n_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["g_lambda_t_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]

    if (is_c3_plus):
        samp_c3_options_yaml["g_eta_slack_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["g_eta_n_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["g_eta_t_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_lambda_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_eta_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_x_position"] = [900] * 3 + [1] * (7*num_objects) + [0.1] * (3 + 6*num_objects)
    else: 
        samp_c3_options_yaml["g_lambda_position_list"] = [[0.005] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["g_x_position"] = [0] * (6 + (13 * num_objects))

    
    samp_c3_options_yaml["u_gamma_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["u_lambda_n_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
    samp_c3_options_yaml["u_lambda_t_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
    if (is_c3_plus):
        samp_c3_options_yaml["u_eta_slack_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["u_eta_n_position_list"] = [[1] * calculate_contacts(num_objects, include_walls * num_objects)]
        samp_c3_options_yaml["u_eta_t_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]   
        samp_c3_options_yaml["u_lambda_position_list"] = [[20] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_eta_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_x_position"] = [0] * (6 + (13 * num_objects))
    else:
        samp_c3_options_yaml["u_lambda_position_list"] = [[1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))]
        samp_c3_options_yaml["u_x_position"] = [10] * 3 + [100, 100, 100, 100, 10, 10, 10] * num_objects + [8] * 3 + [1] * (6*num_objects)

    save_yaml(samp_c3_options_yaml_path, samp_c3_options_yaml)

if __name__ == "__main__":
    # Config paths
    controller_yaml_path = "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
    config = load_yaml(controller_yaml_path)
    vis_yaml_path = config["vis_params_file"]
    sim_yaml_path = config["sim_params_file"]
    goal_yaml_path = config["goal_params_file"]
    sampling_yaml_path = config["sampling_params_file"]
    repos_yaml_path = config["reposition_params_file"]
    lcm_sim_yaml_path = config["lcm_channels_simulation_file"]
    lcm_hardware_yaml_path = config["lcm_channels_hardware_file"]
    samp_c3_options_yaml_path = config["sampling_c3_options_file"]
    goal_yaml_path = config["goal_params_file"]
    repos_yaml_path = config["reposition_params_file"]

    urdf_dir = "examples/sampling_c3/urdf"

    # Load config and get base_names
    config = load_yaml(controller_yaml_path)
    base_names = config.get("base_names", [])
    print(base_names)
    if not base_names:
        print("❌ No base_names found in config.")
        sys.exit(1)



   
    # Process all objects into sdfs
    for i in range(len(base_names)):
        process_obj(
            base_names[i], urdf_dir,
            controller_yaml_path, vis_yaml_path, sim_yaml_path,
            goal_yaml_path, sampling_yaml_path, repos_yaml_path,
            samp_c3_options_yaml_path, i
        )


    # Load YAMLs once to pre-size vectors
    controller_yaml = load_yaml(controller_yaml_path)
    vis_yaml = load_yaml(vis_yaml_path)
    sim_yaml = load_yaml(sim_yaml_path)
    lcm_sim_yaml = load_yaml(lcm_sim_yaml_path)
    lcm_hardware_yaml = load_yaml(lcm_hardware_yaml_path)
    goal_yaml = load_yaml(goal_yaml_path)
    repos_yaml = load_yaml(repos_yaml_path)
    sampling_yaml = load_yaml(sampling_yaml_path)

    controller_yaml["object_models"] = [""] * num_objects
    vis_yaml["object_vis_models"] = [""] * num_objects
    sim_yaml["object_models"] = [""] * num_objects

    for i in range(num_objects): 
        output_dir = os.path.join(urdf_dir, base_names[i])
        set_object_paths(i, base_names[i], output_dir, controller_yaml, vis_yaml, sim_yaml)
    
    lcm_sim_yaml["object_state_channels"] = [f"OBJECT_{name}_STATE_SIMULATION" for name in base_names]
    lcm_hardware_yaml["object_state_channels"] = [f"OBJECT_{name}_STATE_SIMULATION" for name in base_names]

    make_walls(samp_c3_options_yaml_path)

    max_zs = [get_max_z_from_obj(os.path.join(urdf_dir, name, f"{name}.obj")) for name in base_names]
    min_zs = [get_min_z_from_obj(os.path.join(urdf_dir, name, f"{name}.obj")) for name in base_names]
    print(min_zs)
    z_height = min_zs.copy()
    for i in range(len(min_zs)):
        z_height[i] = -0.029 - min_zs[i]

    # set init and goal poses
    sim_yaml["q_init_objects"] = [[0.393, 0, 0, 0.92, 0.4 + (0.04 * index), -0.3 + (0.2 * index), 0.0] for index in range(num_objects)]
    goal_yaml["fixed_target_positions"] = [[0.45, -0.1 + (0.2 * (index % num_objects)), 
                                                z_height[(index-2)]] for index in range(2, num_objects+2)]
    goal_yaml["fixed_target_orientations"] = [[0.707, 0, 0, 0.707] for _ in range(num_objects)]


    goal_yaml["resting_object_heights"] = z_height.copy()
    max_z = max(max_zs)
    min_z = min(min_zs)
    repos_yaml['pwl_waypoint_height'] = float(-0.029 + (max_z - min_z) + 0.05)
    
    heights = min_zs
    max_zs_world = heights
    for i in range(len(max_zs)):
        heights[i] = max_zs[i] - min_zs[i]
        max_zs_world[i] = -0.029 + heights[i]
    
    min_max_z = min(max_zs_world)

    sampling_yaml['z_height'] = max(-0.001, (-0.029 + min_max_z) / 2 + 0.008)
    
    # Update c3_options
    is_c3_plus = "plus" in samp_c3_options_yaml_path
    update_c3_options(is_c3_plus, samp_c3_options_yaml_path)
    print(f"is_c3_plus: {is_c3_plus}")


    # Save pre-sized YAMLs
    save_yaml(controller_yaml_path, controller_yaml)
    save_yaml(vis_yaml_path, vis_yaml)
    save_yaml(sim_yaml_path, sim_yaml)
    save_yaml(lcm_sim_yaml_path, lcm_sim_yaml)
    save_yaml(lcm_hardware_yaml_path, lcm_hardware_yaml)
    save_yaml(repos_yaml_path, repos_yaml)
    save_yaml(goal_yaml_path, goal_yaml)
    save_yaml(sampling_yaml_path, sampling_yaml)

    print("🎉 All objects processed successfully.")