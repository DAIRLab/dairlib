import subprocess
import sys
import os
import re
import trimesh
from ruamel.yaml import YAML
from examples.sampling_c3.sampling_generation.controller_sdf_generation import make_sdf
from examples.sampling_c3.sampling_generation.obj_to_drake_sdf import main as obj_to_drake_sdf
from examples.sampling_c3.sampling_generation.obj_z_planes import get_min_z_from_obj, get_max_z_from_obj


yaml_io = YAML()
yaml_io.preserve_quotes = True
yaml_io.indent(sequence=4, offset=2)
yaml_io.width = 1000

def coarsify_obj(path): # Returns path of object, if it coarsified it
    mesh = trimesh.load(path)  
    num_faces = len(mesh.faces)
    if (num_faces > 5000):
        ratio = 1 - (5000 / num_faces)
        simplified = mesh.simplify_quadric_decimation(ratio)
        base, ext = os.path.splitext(path)
        new_path = base + "_coarse" + ext
        simplified.export(new_path)
        return new_path, True
    else:
        return path, False

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

def parse_max_z(stdout):
    match = re.search(r"z\s*=\s*([-+]?\d*\.\d+|\d+)", stdout)
    if match:
        return float(match.group(1))
    else:
        raise ValueError("Could not find 'z =' line in obj_max_z_plane output.")


if __name__ == "__main__":
    config_path = "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
    config = load_yaml(config_path)

    base_name = config["base_name"]
    output_dir = f"examples/sampling_c3/urdf/{base_name}"
    os.makedirs(output_dir, exist_ok=True)

    obj_dir = f"examples/sampling_c3/urdf"
    obj_file = os.path.join(obj_dir, f"{base_name}.obj")
    coarse_path, is_coarse = coarsify_obj(obj_file)

    coarse = "_coarse" if is_coarse else ""

    convex_name = f"{base_name}_convex"
    controller_sdf_path = os.path.join(output_dir, f"{base_name}{coarse}_controller.sdf")
    combined_sdf_path = os.path.join(output_dir, f"{base_name}{coarse}.sdf")

    os.makedirs(output_dir, exist_ok=True)

    # Copy file using shell cp
    os.system(f"cp {obj_file} {output_dir}/")
    print(f"Copied {obj_file} to {output_dir}/")

    # Generate the sim SDFs
    obj_to_drake_sdf(coarse_path, output_dir)

    # Generate controller SDF
    make_sdf(coarse_path, controller_sdf_path)

    # Find the min z height plane
    min_z_output = get_min_z_from_obj(coarse_path)
    print(f"Parsed min_z_output: {min_z_output:.6f}")

    # Find the max z height plane
    max_z_output = get_max_z_from_obj(coarse_path)
    print(f"Parsed max_z_output: {max_z_output:.6f}")

    # --- YAML UPDATES ---
    # Update controller object
    controller_yaml_path = "examples/sampling_c3/anything/parameters/sampling_c3_controller_params.yaml"
    controller_yaml = load_yaml(controller_yaml_path)
    controller_yaml["object_model"] = controller_sdf_path
    save_yaml(controller_yaml_path, controller_yaml)

    # Update sim/visualizer object 
    vis_yaml_path = "examples/sampling_c3/anything/parameters/vis_params.yaml"
    sim_yaml_path = "examples/sampling_c3/anything/parameters/sim_params.yaml"

    vis_yaml = load_yaml(vis_yaml_path)
    vis_yaml["object_vis_model"] = combined_sdf_path

    sim_yaml = load_yaml(sim_yaml_path)
    sim_yaml["object_model"] = combined_sdf_path

    if "q_init_object" in sim_yaml and isinstance(sim_yaml["q_init_object"], list):
        sim_yaml["q_init_object"][-1] = float(-0.029-min_z_output)
    
    save_yaml(vis_yaml_path, vis_yaml)
    save_yaml(sim_yaml_path, sim_yaml)

    # Set goal z-height based on object 
    goal_yaml_path = "examples/sampling_c3/anything/parameters/goal_params.yaml"
    goal_yaml = load_yaml(goal_yaml_path)
    goal_yaml["resting_object_height"] = float(-0.029-min_z_output)
    if "fixed_target_position" in goal_yaml and isinstance(goal_yaml["fixed_target_position"], list):
        goal_yaml["fixed_target_position"][-1] = float(-0.029-min_z_output)
    save_yaml(goal_yaml_path, goal_yaml)

    # Set sample z-height to middle of object z range
    sampling_yaml_path = "examples/sampling_c3/anything/parameters/sampling_params.yaml"
    sampling_yaml = load_yaml(sampling_yaml_path)
    sampling_yaml["z_height"] = float(-0.029 + (max_z_output-min_z_output)/2 +0.01)
    if "z_height" in sampling_yaml and isinstance(sampling_yaml["z_height"], list):
        sampling_yaml["z_height"][-1] = float(-0.029-min_z_output)
    save_yaml(sampling_yaml_path, sampling_yaml)

    # Set ee repositioning height based on object height
    repos_yaml_path = "examples/sampling_c3/anything/parameters/reposition_params.yaml"
    repos_yaml = load_yaml(repos_yaml_path)
    repos_yaml["pwl_waypoint_height"] = float(-0.029 + max_z_output + 0.05)
    if "pwl_waypoint_height" in repos_yaml and isinstance(repos_yaml["pwl_waypoint_height"], list):
        repos_yaml["pwl_waypoint_height"][-1] = float(-0.029-min_z_output)
    save_yaml(repos_yaml_path, repos_yaml)

    print("\nAll YAML files updated and pipeline completed successfully.")