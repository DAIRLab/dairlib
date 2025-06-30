import subprocess
import sys
import os
import re
from ruamel.yaml import YAML

yaml_io = YAML()
yaml_io.preserve_quotes = True
yaml_io.indent(sequence=4, offset=2)
yaml_io.width = 1000

def run_command(cmd, description):
    print(f"\n▶️ Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed: {description}")
        print(e.stderr)
        sys.exit(1)

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml_io.load(f)

def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml_io.dump(data, f)

def parse_min_z(stdout):
    match = re.search(r"z\s*=\s*([-+]?\d*\.\d+|\d+)", stdout)
    if match:
        return float(match.group(1))
    else:
        raise ValueError("Could not find 'z =' line in obj_min_z_plane output.")

if __name__ == "__main__":
    config_path = "examples/sampling_c3/push_t/parameters/sampling_params.yaml"
    config = load_yaml(config_path)

    base_name = config["base_name"]
    output_dir = "examples/sampling_c3/script_output"
    os.makedirs(output_dir, exist_ok=True)

    obj_dir = "examples/sampling_c3/script_output"
    obj_file = os.path.join(obj_dir, f"{base_name}.obj")
    convex_name = f"{base_name}_convex"
    controller_sdf_path = os.path.join(output_dir, f"{base_name}_controller.sdf")
    combined_sdf_path = os.path.join(output_dir, f"{base_name}.sdf")

    # 1. controller_sdf_generation.py
    run_command(
        ["python", "examples/sampling_c3/sampling_generation/controller_sdf_generation.py", obj_file, controller_sdf_path],
        "Controller SDF generation"
    )

    # 2. obj_to_drake_sdf.py
    run_command(
        ["python", "examples/sampling_c3/sampling_generation/obj_to_drake_sdf.py", obj_file, output_dir],
        "OBJ to Drake SDF"
    )

    # 3. combine_obj_files.py
    run_command(
        ["python", "examples/sampling_c3/sampling_generation/combine_script.py", convex_name, output_dir],
        "Combining convex OBJ files"
    )

    # 4. obj_min_z_plane.py
    z_output = run_command(
        ["python", "examples/sampling_c3/sampling_generation/obj_min_z_plane.py", obj_file],
        "Extracting min z-height"
    )
    min_z_height = parse_min_z(z_output)
    print(f"✅ Parsed min_z_height: {min_z_height:.6f}")

    # --- YAML UPDATES ---
    # 1. franka_c3_controller_params.yaml
    controller_yaml_path = "examples/sampling_c3/push_t/parameters/franka_c3_controller_params.yaml"
    controller_yaml = load_yaml(controller_yaml_path)
    controller_yaml["jack_model"] = controller_sdf_path
    save_yaml(controller_yaml_path, controller_yaml)

    # 2. franka_sim_params.yaml
    sim_yaml_path = "examples/sampling_c3/push_t/parameters/franka_sim_params.yaml"
    sim_yaml = load_yaml(sim_yaml_path)
    sim_yaml["jack_model"] = combined_sdf_path
    if "q_init_object" in sim_yaml and isinstance(sim_yaml["q_init_object"], list):
        sim_yaml["q_init_object"][-1] = float(-0.029-min_z_height)
    save_yaml(sim_yaml_path, sim_yaml)

    # 3. trajectory_params.yaml
    traj_yaml_path = "examples/sampling_c3/push_t/parameters/trajectory_params.yaml"
    traj_yaml = load_yaml(traj_yaml_path)
    traj_yaml["resting_object_height"] = float(-0.029-min_z_height)
    if "fixed_target_position" in traj_yaml and isinstance(traj_yaml["fixed_target_position"], list):
        traj_yaml["fixed_target_position"][-1] = float(-0.029-min_z_height)
    save_yaml(traj_yaml_path, traj_yaml)

    print("\n✅ All YAML files updated and pipeline completed successfully.")