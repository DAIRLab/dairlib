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

# C3+ options configs for different number of objects
NUM_OUTER_THREADS_CONFIGS = {1: 6, 2: 5, 3: 4, 4: 5}
PLANNING_HORIZON_CONFIGS = {1: 10, 2: 15, 3: 7, 4: 7}

W_Q_CONFIGS = {1: 80, 2: 80, 3: 80, 4: 80}
W_R_CONFIGS = {1: 6, 2: 6, 3: 10, 4: 10}
W_G_CONFIGS = {1: 0.18, 2: 0.18, 3: 0.15, 4: 0.2}
W_U_CONFIGS = {1: 0.5, 2: 0.5, 3: 0.5, 4: 0.5}

W_Q_POSITION_CONFIGS = {1: 50, 2: 50, 3: 50, 4: 50}
W_R_POSITION_CONFIGS = {1: 6, 2: 6, 3: 10, 4: 10}
W_G_POSITION_CONFIGS = {1: 0.18, 2: 0.18, 3: 0.15, 4: 0.2}
W_U_POSITION_CONFIGS = {1: 0.26, 2: 0.26, 3: 0.26, 4: 0.26}

INCLUDE_WALLS_CONFIGS = {1: 1, 2: 1, 3: 1, 4: 0}

# Progress params configs for different number of objects
COST_SWITCHING_THRESHOLD_DISTANCE_CONFIGS = {1: 0.05, 2: 0.05, 3: 0.05, 4: 0.05}
PROGRESS_ENFORCED_OVER_N_LOOPS_CONFIGS = {1: 35, 2: 25, 3: 18, 4: 16}

# Sampling params configs for different number of objects
NUM_ADDITIONAL_SAMPLES_REPOS_CONFIGS = {1: 4, 2: 3, 3: 2, 4: 3}
NUM_ADDITIONAL_SAMPLES_C3_CONFIGS = {1: 5, 2: 4, 3: 3, 4: 4}


def coarsify_obj(path):
    mesh = trimesh.load(path)
    num_faces = len(mesh.faces)
    if num_faces > 10000:
        print("COARSIFIED")
        ratio = 1 - (10000 / num_faces)

        root, ext = os.path.splitext(path)
        new_path = f"{root}_backup{ext}"

        mesh.export(new_path)  # make backup of object
        simplified = mesh.simplify_quadric_decimation(ratio)
        simplified.export(path)
        return True
    return False


def load_yaml(path):
    with open(path, "r") as f:
        return yaml_io.load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml_io.dump(data, f)


def get_num_objects_from_yaml(yaml_path: str) -> int:
    with open(yaml_path, "r") as f:
        data = yaml_io.load(f)
    models = data.get("base_names")
    return len(models)


def compute_num_contacts_obj_to_obj(num_objects: int) -> int:
    return choose_2(num_objects)


def compute_num_contacts_obj_to_ground(num_objects: int) -> int:
    return num_objects * 3


def calculate_contacts(num_objects: int, include_walls: int) -> int:
    return (
        compute_num_contacts_obj_to_obj(num_objects)
        + compute_num_contacts_obj_to_ground(num_objects)
        + 1
        + include_walls
    )


def choose_2(num_objects: int) -> int:
    return int(num_objects * (num_objects - 1) // 2)


yaml_path = (
    "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
)
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
    index: int,
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


def build_mu_per_pair_type(num_objects: int, include_walls: int) -> list:
    EE_GROUND_FRICTION_COEFFICIENT = 0.823
    EE_OBJECT_FRICTION_COEFFICIENT = 0.42
    OBJECT_GROUND_FRICTION_COEFFICIENT = 0.46
    OBJECT_OBJECT_FRICTION_COEFFICIENT = 0.3
    OBJECT_WALL_FRICTION_COEFFICIENT = 0.375

    mu_per_pair_type = [
        EE_GROUND_FRICTION_COEFFICIENT,
        EE_OBJECT_FRICTION_COEFFICIENT,
        OBJECT_GROUND_FRICTION_COEFFICIENT,
    ] + [OBJECT_OBJECT_FRICTION_COEFFICIENT] * choose_2(num_objects)
    if include_walls:
        mu_per_pair_type += [OBJECT_WALL_FRICTION_COEFFICIENT] * num_objects
    return mu_per_pair_type


# Build q_vector for n objects
def build_q_vector(num_objects: int, is_full_pose_tracking: bool = True) -> list:
    EE_POSITION = [0.01, 0.01, 0.01]

    if is_full_pose_tracking:
        OBJECT_ORIENTATION = [0.1, 0.1, 0.1, 0.1]
        OBJECT_POSITION = [150, 150, 120]
    else:
        OBJECT_ORIENTATION = [0, 0, 0, 0]
        OBJECT_POSITION = [200, 200, 120]

    EE_LINEAR_VELOCITY = [15, 15, 10]
    OBJECT_ANGULAR_VELOCITY = [0.05, 0.05, 0.05]
    OBJECT_LINEAR_VELOCITY = [0.05, 0.05, 0.05]

    q_vector = []

    # 1. EE position
    q_vector.extend(EE_POSITION)

    # 2. Object orientations & positions
    for _ in range(num_objects):
        q_vector.extend(OBJECT_ORIENTATION)
        q_vector.extend(OBJECT_POSITION)

    # 3. EE linear velocity\
    q_vector.extend(EE_LINEAR_VELOCITY)

    # 4. Object angular & linear velocities
    for _ in range(num_objects):
        q_vector.extend(OBJECT_ANGULAR_VELOCITY)
        q_vector.extend(OBJECT_LINEAR_VELOCITY)

    return q_vector


def update_c3_options(is_c3_plus, samp_c3_options_yaml_path):
    samp_c3_options_yaml = load_yaml(samp_c3_options_yaml_path)
    include_walls = INCLUDE_WALLS_CONFIGS[num_objects]
    samp_c3_options_yaml["include_walls"] = True if include_walls else False
    samp_c3_options_yaml["N"] = PLANNING_HORIZON_CONFIGS[num_objects]
    samp_c3_options_yaml["num_outer_threads"] = NUM_OUTER_THREADS_CONFIGS[num_objects]
    samp_c3_options_yaml["w_Q"] = W_Q_CONFIGS[num_objects]
    samp_c3_options_yaml["w_R"] = W_R_CONFIGS[num_objects]
    samp_c3_options_yaml["w_G"] = W_G_CONFIGS[num_objects]
    samp_c3_options_yaml["w_U"] = W_U_CONFIGS[num_objects]
    samp_c3_options_yaml["w_Q_position"] = W_Q_POSITION_CONFIGS[num_objects]
    samp_c3_options_yaml["w_R_position"] = W_R_POSITION_CONFIGS[num_objects]
    samp_c3_options_yaml["w_G_position"] = W_G_POSITION_CONFIGS[num_objects]
    samp_c3_options_yaml["w_U_position"] = W_U_POSITION_CONFIGS[num_objects]

    samp_c3_options_yaml["resolve_contacts_to_lists"] = [
        [0, 1, num_objects * 3]
        + [1] * choose_2(num_objects)
        + [include_walls] * num_objects
    ]
    samp_c3_options_yaml["resolve_as_planar_contacts_list"] = (
        [0, 0, 0] + [1] * choose_2(num_objects) + [include_walls] * num_objects
    )
    samp_c3_options_yaml["mu_per_pair_type"] = build_mu_per_pair_type(
        num_objects, include_walls
    )

    samp_c3_options_yaml["q_vector"] = build_q_vector(
        num_objects, is_full_pose_tracking=True
    )

    q_vector_position = build_q_vector(num_objects, is_full_pose_tracking=False)
    q_vector_position[3 + 7 * num_objects] *= 1
    q_vector_position[4 + 7 * num_objects] *= 1
    q_vector_position[5 + 7 * num_objects] *= 1

    samp_c3_options_yaml["q_vector_position"] = q_vector_position

    samp_c3_options_yaml["g_x"] = (
        [950] * 3 + [1] * (7 * num_objects) + [0.1] * (3 + 6 * num_objects)
    )
    samp_c3_options_yaml["g_gamma_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["g_lambda_n_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["g_lambda_t_list"] = [
        [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
    ]

    if is_c3_plus:
        samp_c3_options_yaml["g_eta_slack_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["g_eta_n_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["g_eta_t_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_eta_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_lambda_list"] = [
            [2] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
    else:
        samp_c3_options_yaml["g_lambda_list"] = [
            [0.05] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]

    samp_c3_options_yaml["u_gamma_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["u_lambda_n_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["u_lambda_t_list"] = [
        [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
    ]

    if is_c3_plus:
        samp_c3_options_yaml["u_eta_slack_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["u_eta_n_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["u_eta_t_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["u_lambda_list"] = [
            [1000] * 4
            + [1000] * 4 * compute_num_contacts_obj_to_ground(num_objects)
            + [1] * 4 * compute_num_contacts_obj_to_obj(num_objects)
            + [1] * (4 * num_objects * include_walls)
        ]
        samp_c3_options_yaml["u_eta_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["u_x"] = [0] * (6 + (13 * num_objects))
    else:
        samp_c3_options_yaml["u_lambda_list"] = [
            [10] * (4 * calculate_contacts(num_objects, 0))
            + [1] * (4 * num_objects * include_walls)
        ]
        samp_c3_options_yaml["u_x"] = (
            [10] * 3
            + [100, 100, 100, 100, 10, 10, 10] * num_objects
            + [8] * 3
            + [1] * (6 * num_objects)
        )

    samp_c3_options_yaml["g_gamma_position_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["g_lambda_n_position_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["g_lambda_t_position_list"] = [
        [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
    ]

    if is_c3_plus:
        samp_c3_options_yaml["g_eta_slack_position_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["g_eta_n_position_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["g_eta_t_position_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_lambda_position_list"] = [
            [2] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_eta_position_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_x_position"] = (
            [950] * 3 + [1] * (7 * num_objects) + [0.1] * (3 + 6 * num_objects)
        )
    else:
        samp_c3_options_yaml["g_lambda_position_list"] = [
            [0.005] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["g_x_position"] = [0] * (6 + (13 * num_objects))

    samp_c3_options_yaml["u_gamma_position_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["u_lambda_n_position_list"] = [
        [1] * calculate_contacts(num_objects, include_walls * num_objects)
    ]
    samp_c3_options_yaml["u_lambda_t_position_list"] = [
        [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
    ]
    if is_c3_plus:
        samp_c3_options_yaml["u_eta_slack_position_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["u_eta_n_position_list"] = [
            [1] * calculate_contacts(num_objects, include_walls * num_objects)
        ]
        samp_c3_options_yaml["u_eta_t_position_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["u_lambda_position_list"] = [
            [1000] * 4
            + [1000] * 4 * compute_num_contacts_obj_to_ground(num_objects)
            + [1] * 4 * compute_num_contacts_obj_to_obj(num_objects)
            + [1] * (4 * num_objects * include_walls)
        ]
        samp_c3_options_yaml["u_eta_position_list"] = [
            [1] * (4 * calculate_contacts(num_objects, include_walls * num_objects))
        ]
        samp_c3_options_yaml["u_x_position"] = [0] * (6 + (13 * num_objects))
    else:
        samp_c3_options_yaml["u_lambda_position_list"] = [
            [1000] * (4 * calculate_contacts(num_objects, 0))
            + [1] * (4 * num_objects * include_walls)
        ]
        samp_c3_options_yaml["u_x_position"] = (
            [10] * 3
            + [100, 100, 100, 100, 10, 10, 10] * num_objects
            + [8] * 3
            + [1] * (6 * num_objects)
        )

    save_yaml(samp_c3_options_yaml_path, samp_c3_options_yaml)


def set_goal_poses(goal_yaml: dict, z_height: list, base_names: list):
    # Load the letter settings yaml.
    letter_settings = load_yaml(
        "examples/sampling_c3/anything/parameters/letter_settings.yaml"
    )

    positions = []
    quaternions = []
    num_objects = len(base_names)
    for i, name in enumerate(base_names):
        print(f"Object {i}: {name}, z_height: {z_height[i]:.6f}")
        positions.append(
            [0.5, 0.2 * (i % num_objects) - (0.2 * (num_objects - 1)) / 2, z_height[i]]
        )
        quaternions.append([0.707, 0, 0, 0.707])
        if name in letter_settings.keys():
            xy_offset, quat = letter_settings[name]
            positions[-1][0] += xy_offset[0]
            positions[-1][1] += xy_offset[1]
            quaternions[-1] = quat

    goal_yaml["fixed_target_positions"] = positions
    goal_yaml["fixed_target_orientations"] = quaternions


def update_sampling_params(sampling_yaml: dict):
    sampling_yaml["num_additional_samples_repos"] = (
        NUM_ADDITIONAL_SAMPLES_REPOS_CONFIGS[num_objects]
    )
    sampling_yaml["num_additional_samples_c3"] = NUM_ADDITIONAL_SAMPLES_C3_CONFIGS[
        num_objects
    ]
    sampling_yaml["z_height"] = 0.002  # max(0.002, (-0.029 + min_max_z) / 2 + 0.008)


def update_progress_params(progress_yaml: dict):
    progress_yaml["cost_switching_threshold_distance"] = (
        COST_SWITCHING_THRESHOLD_DISTANCE_CONFIGS[num_objects]
    )
    progress_yaml["progress_enforced_over_n_loops"] = (
        PROGRESS_ENFORCED_OVER_N_LOOPS_CONFIGS[num_objects]
    )


def main():
    # Config paths
    controller_yaml_path = (
        "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
    )
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
    progress_yaml_path = config["progress_params_file"]

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
            base_names[i],
            urdf_dir,
            controller_yaml_path,
            vis_yaml_path,
            sim_yaml_path,
            goal_yaml_path,
            sampling_yaml_path,
            repos_yaml_path,
            samp_c3_options_yaml_path,
            i,
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
    progress_yaml = load_yaml(progress_yaml_path)
    controller_yaml["object_models"] = [""] * num_objects
    vis_yaml["object_vis_models"] = [""] * num_objects
    sim_yaml["object_models"] = [""] * num_objects

    for i in range(num_objects):
        output_dir = os.path.join(urdf_dir, base_names[i])
        set_object_paths(
            i, base_names[i], output_dir, controller_yaml, vis_yaml, sim_yaml
        )

    lcm_sim_yaml["object_state_channels"] = [
        f"OBJECT_{name}_STATE_SIMULATION" for name in base_names
    ]
    lcm_hardware_yaml["object_state_channels"] = [
        f"OBJECT_{name}_STATE_SIMULATION" for name in base_names
    ]

    max_zs = [
        get_max_z_from_obj(os.path.join(urdf_dir, name, f"{name}.obj"))
        for name in base_names
    ]
    min_zs = [
        get_min_z_from_obj(os.path.join(urdf_dir, name, f"{name}.obj"))
        for name in base_names
    ]
    print(min_zs)
    z_height = min_zs.copy()
    for i in range(len(min_zs)):
        z_height[i] = -0.029 - min_zs[i]

    # set init and goal poses
    sim_yaml["q_init_objects"] = [
        [0.393, 0, 0, 0.92, 0.4 + (0.02 * index), -0.3 + (0.2 * index), 0.0]
        for index in range(num_objects)
    ]
    set_goal_poses(goal_yaml, z_height, base_names)

    goal_yaml["resting_object_heights"] = z_height.copy()
    max_z = max(max_zs)
    min_z = min(min_zs)
    repos_yaml["pwl_waypoint_height"] = float(-0.029 + (max_z - min_z) + 0.05)

    heights = min_zs
    max_zs_world = heights
    for i in range(len(max_zs)):
        heights[i] = max_zs[i] - min_zs[i]
        max_zs_world[i] = -0.029 + heights[i]

    min_max_z = min(max_zs_world)


    # Update c3_options
    is_c3_plus = "plus" in samp_c3_options_yaml_path
    update_c3_options(is_c3_plus, samp_c3_options_yaml_path)
    print(f"is_c3_plus: {is_c3_plus}")

    update_sampling_params(sampling_yaml)
    update_progress_params(progress_yaml)

    # Save pre-sized YAMLs
    save_yaml(controller_yaml_path, controller_yaml)
    save_yaml(vis_yaml_path, vis_yaml)
    save_yaml(sim_yaml_path, sim_yaml)
    save_yaml(lcm_sim_yaml_path, lcm_sim_yaml)
    save_yaml(lcm_hardware_yaml_path, lcm_hardware_yaml)
    save_yaml(repos_yaml_path, repos_yaml)
    save_yaml(goal_yaml_path, goal_yaml)
    save_yaml(sampling_yaml_path, sampling_yaml)
    save_yaml(progress_yaml_path, progress_yaml)
    print("🎉 All objects processed successfully.")


if __name__ == "__main__":
    main()
