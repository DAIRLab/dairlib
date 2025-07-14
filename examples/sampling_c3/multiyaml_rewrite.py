import sys, os, re, trimesh
from ruamel.yaml import YAML
from examples.sampling_c3.sampling_generation.controller_sdf_generation import make_sdf
from examples.sampling_c3.sampling_generation.obj_to_drake_sdf import main as obj_to_drake_sdf
from examples.sampling_c3.sampling_generation.obj_z_planes import get_min_z_from_obj, get_max_z_from_obj

yaml_io = YAML()
yaml_io.preserve_quotes = True
yaml_io.indent(sequence=4, offset=2)
yaml_io.width = 1000
yaml_io.default_flow_style = True

def coarsify_obj(path):
    mesh = trimesh.load(path)
    num_faces = len(mesh.faces)
    if num_faces > 5000:
        ratio = 1 - (5000/num_faces)
        simplified = mesh.simplify_quadric_decimation(ratio)
        base, ext = os.path.splitext(path)
        new_path = base + "_coarse" + ext
        simplified.export(new_path)
        return new_path, True
    return path, False

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

def calculate_contacts(num_objects: int) -> int:
    return num_objects * (num_objects - 1) // 2 + num_objects * 3 + 1

yaml_path = "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
num_objects = get_num_objects_from_yaml(yaml_path)
num_contacts = calculate_contacts(num_objects)
print("Number of objects:", num_objects)
print("Number of contacts:", num_contacts)
def process_base(
    base_name: str,
    urdf_dir: str,
    controller_yaml_path: str,
    vis_yaml_path: str,
    sim_yaml_path: str,
    goal_yaml_path: str,
    sampling_yaml_path: str,
    repos_yaml_path: str
):
    print(f"\n🚀 Processing object: {base_name}")
    output_dir = os.path.join(urdf_dir, base_name)
    os.makedirs(output_dir, exist_ok=True)

    obj_file = os.path.join(urdf_dir, f"{base_name}.obj")
    coarse_path, is_coarse = coarsify_obj(obj_file)

    # Copy original OBJ to output directory
    os.system(f"cp {obj_file} {output_dir}/")
    print(f"📦 Copied {obj_file} → {output_dir}/")

    # Create SDF file paths
    coarse_suffix = "_coarse" if is_coarse else ""
    controller_sdf_path = os.path.join(output_dir, f"{base_name}{coarse_suffix}_controller.sdf")
    combined_sdf_path = os.path.join(output_dir, f"{base_name}{coarse_suffix}.sdf")

    # Generate SDFs
    obj_to_drake_sdf(coarse_path, output_dir)
    make_sdf(coarse_path, controller_sdf_path)

    # Get min/max z-heights
    min_z = get_min_z_from_obj(coarse_path)
    max_z = get_max_z_from_obj(coarse_path)
    print(f"✅ {base_name} → min_z={min_z:.6f}, max_z={max_z:.6f}")

    index = base_names.index(base_name)

    # --- YAML Updates ---
    # 1. Update controller YAML
    controller_yaml = load_yaml(controller_yaml_path)
    controller_yaml["object_models"][index] = controller_sdf_path
    save_yaml(controller_yaml_path, controller_yaml)

    # 2. Update visualization YAML
    vis_yaml = load_yaml(vis_yaml_path)
    vis_yaml["object_vis_models"][index] = combined_sdf_path
    save_yaml(vis_yaml_path, vis_yaml)

    # 3. Update simulation YAML
    sim_yaml = load_yaml(sim_yaml_path)
    sim_yaml["object_models"][index] = combined_sdf_path
    sim_yaml["q_init_objects"][index][6] = float(-0.029 - min_z)
    save_yaml(sim_yaml_path, sim_yaml)

    # # 4. Update goal YAML
    # goal_yaml = load_yaml(goal_yaml_path)
    # goal_yaml["resting_object_height"] = float(-0.029 - min_z)
    # if "fixed_target_position" in goal_yaml and isinstance(goal_yaml["fixed_target_position"], list):
    #     goal_yaml["fixed_target_position"][-1] = float(-0.029 - min_z)
    # save_yaml(goal_yaml_path, goal_yaml)

    # # 5. Update sampling YAML
    # sampling_yaml = load_yaml(sampling_yaml_path)
    # sampling_yaml["z_height"] = float(-0.029 + (max_z - min_z) / 2 + 0.01)
    # save_yaml(sampling_yaml_path, sampling_yaml)

    # # 6. Update reposition YAML
    # repos_yaml = load_yaml(repos_yaml_path)
    # repos_yaml["pwl_waypoint_height"] = float(-0.029 + max_z + 0.05)
    # save_yaml(repos_yaml_path, repos_yaml)

    # 7. Update LCM simulation channels done in main loop instead of here

    # 8. Update sampling C3 options

    samp_c3_options_yaml = load_yaml(samp_c3_options_yaml_path)
    samp_c3_options_yaml["resolve_contacts_to_lists"][0][2] = num_objects * 3

    EE_POSITION = [0.01, 0.01, 0.01]
    OBJECT_ORIENTATION = [0.1, 0.1, 0.1, 0.1]
    OBJECT_POSITION = [500, 500, 120]
    EE_LINEAR_VELOCITY = [5, 5, 5]
    OBJECT_ANGULAR_VELOCITY = [0.05, 0.05, 0.05]
    OBJECT_LINEAR_VELOCITY = [0.05, 0.05, 0.05]

    # Build q_vector for n objects
    def build_q_vector(num_objects: int) -> list:
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

    samp_c3_options_yaml["q_vector"] = build_q_vector(num_objects)
    samp_c3_options_yaml["q_vector_position"] = build_q_vector(num_objects)

    samp_c3_options_yaml["g_gamma_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["g_lambda_n_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["g_lambda_t_list"] = [[1] * (4 * (num_objects * 3 + 1))]
    samp_c3_options_yaml["g_lambda_list"] = [[0.005] * (4 * (num_objects * 3 + 1))]

    samp_c3_options_yaml["u_gamma_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["u_lambda_n_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["u_lambda_t_list"] = [[1] * (4 * (num_objects * 3 + 1))]
    samp_c3_options_yaml["u_lambda_list"] = [[10] * (4 * (num_objects * 3 + 1))]

    samp_c3_options_yaml["g_gamma_position_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["g_lambda_n_position_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["g_lambda_t_position_list"] = [[1] * (4 * (num_objects * 3 + 1))]
    samp_c3_options_yaml["g_lambda_position_list"] = [[0.005] * (4 * (num_objects * 3 + 1))]

    samp_c3_options_yaml["u_gamma_position_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["u_lambda_n_position_list"] = [[1] * (num_objects * 3 + 1)]
    samp_c3_options_yaml["u_lambda_t_position_list"] = [[1] * (4 * (num_objects * 3 + 1))]
    samp_c3_options_yaml["u_lambda_position_list"] = [[10] * (4 * (num_objects * 3 + 1))]

    save_yaml(samp_c3_options_yaml_path, samp_c3_options_yaml)

    print(f"✅ Finished processing {base_name}\n")

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
    samp_c3_options_yaml_path = config["sampling_c3_options_file"]

    urdf_dir = "examples/sampling_c3/urdf"

    # Load config and get base_names
    config = load_yaml(controller_yaml_path)
    base_names = config.get("base_names", [])
    if not base_names:
        print("❌ No base_names found in config.")
        sys.exit(1)

    # Load YAMLs once to pre-size vectors
    controller_yaml = load_yaml(controller_yaml_path)
    vis_yaml = load_yaml(vis_yaml_path)
    sim_yaml = load_yaml(sim_yaml_path)
    lcm_sim_yaml = load_yaml(lcm_sim_yaml_path)

    # Zero vectors only once
    controller_yaml["object_models"] = [""] * num_objects
    vis_yaml["object_vis_models"] = [""] * num_objects
    sim_yaml["object_models"] = [""] * num_objects
    sim_yaml["q_init_objects"] = [[0, 0, 0, 1, 0.5, 0.0, 0.0] for _ in range(num_objects)]
    lcm_sim_yaml["object_state_channels"] = [f"OBJECT_{name}_STATE_SIMULATION" for name in base_names]

    # Save pre-sized YAMLs
    save_yaml(controller_yaml_path, controller_yaml)
    save_yaml(vis_yaml_path, vis_yaml)
    save_yaml(sim_yaml_path, sim_yaml)

    # Process all objects
    for base_name in base_names:
        process_base(
            base_name, urdf_dir,
            controller_yaml_path, vis_yaml_path, sim_yaml_path,
            goal_yaml_path, sampling_yaml_path, repos_yaml_path
        )

    print("🎉 All objects processed successfully.")