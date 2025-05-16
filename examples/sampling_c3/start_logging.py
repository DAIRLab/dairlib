import subprocess
import os
import glob
import codecs
from datetime import date
import sys
import yaml

def main(log_type, example_name):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"/mnt/data2/sharanya/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/dairlib/"

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    # franka_cr_controller params path
    franka_c3_controller_params_path = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "franka_c3_controller_params.yaml"
    )

    with open(franka_c3_controller_params_path) as f:
        franka_c3_controller_params = yaml.load(f, Loader=yaml.FullLoader)

    c3_gains = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "franka_c3_options_floating.yaml"
    )
    sampling_c3_options = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "sampling_c3_options.yaml"
    )
    sampling_params = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "sampling_params.yaml"
    )

    osc_gains = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "franka_osc_controller_params.yaml"
    )
    sim_params = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "franka_sim_params.yaml"
    )
    trajectory_params = os.path.join(
        dair, "examples", "sampling_c3", example_name, "parameters", 
        "trajectory_params.yaml"
    )
    ee_simple_model_urdf = dair + "examples/sampling_c3/urdf/end_effector_simple_model.urdf"

    object_sdf_dir = os.path.join(dair, "examples", "sampling_c3", "urdf")

    if example_name == 'jacktoy':
        object_sim_sdf = os.path.join(object_sdf_dir, "jack.sdf")
        object_c3_sdf = os.path.join(object_sdf_dir, "jack_ground.sdf")
    elif example_name == 'push_t':
        object_sim_sdf = os.path.join(object_sdf_dir, "T_vertical_sim.sdf")
        object_c3_sdf = os.path.join(object_sdf_dir, "T_vertical.sdf")
    elif example_name == 'box_topple':
        object_sim_sdf = os.path.join(object_sdf_dir, "box_sim.sdf")
        object_c3_sdf = os.path.join(object_sdf_dir, "box.sdf")
    elif example_name == 'ball_rolling':
        object_sim_sdf = os.path.join(object_sdf_dir, "ball.sdf")
        object_c3_sdf = os.path.join(object_sdf_dir, "ball.sdf")
    else:
        raise ValueError(f"Unknown example_name '{example_name}'. "
                         "Supported examples: jacktoy, push_t, box_topple, ball_rolling")

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)
    # current_logs = sorted(glob.glob('*'))
    # try:
    #     last_log = int(current_logs[-1].split('-')[-1])


    try:
        directories = glob.glob(os.path.join(logdir, "*"))    
        directory_names = [os.path.basename(d) for d in directories if os.path.isdir(d)]
        last_log = max([int(name) for name in directory_names if name.isdigit()])
        log_num = str(last_log+1).zfill(6)
    except:
        log_num = str(0).zfill(6)

    if log_type == 'hw':
        with open('commit_tag%s' % log_num, 'w') as f:
            f.write(str(commit_tag))
            f.write("\n\ngit diff:\n\n")
            f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
    if not os.path.isdir(log_num):
        os.mkdir(log_num)
    
    os.chdir(log_num)
    logname = f'{log_type}log-{log_num}'
    subprocess.run(['cp', franka_c3_controller_params_path, f'franka_c3_controller_params_{log_num}.yaml'])
    subprocess.run(['cp', osc_gains, f'osc_gains_{log_num}.yaml'])
    subprocess.run(['cp', sim_params, f'sim_params_{log_num}.yaml'])
    subprocess.run(['cp', c3_gains, f'c3_gains_{log_num}.yaml'])
    subprocess.run(['cp', sampling_c3_options, f'sampling_c3_options_{log_num}.yaml'])
    subprocess.run(['cp', sampling_params, f'sampling_params_{log_num}.yaml'])
    subprocess.run(['cp', trajectory_params, f'trajectory_params_{log_num}.yaml'])
    subprocess.run(['cp', ee_simple_model_urdf, f'ee_simple_model_urdf_{log_num}.urdf'])
    if example_name == 'jacktoy':
        subprocess.run(['cp', object_sim_sdf, f'jack_sim_sdf_{log_num}.sdf'])
        subprocess.run(['cp', object_c3_sdf, f'jack_c3_sdf_{log_num}.sdf'])
    elif example_name == 'push_t':
        subprocess.run(['cp', object_sim_sdf, f't_sim_sdf_{log_num}.sdf'])
        subprocess.run(['cp', object_c3_sdf, f't_c3_sdf_{log_num}.sdf'])
    elif example_name == 'box_topple':
        subprocess.run(['cp', object_sim_sdf, f'box_sim_sdf_{log_num}.sdf'])
        subprocess.run(['cp', object_c3_sdf, f'box_c3_sdf_{log_num}.sdf'])
    elif example_name == 'ball_rolling':
        subprocess.run(['cp', object_sim_sdf, f'ball_sim_sdf_{log_num}.sdf'])
        subprocess.run(['cp', object_c3_sdf, f'ball_c3_sdf_{log_num}.sdf'])
    else:
        raise ValueError(f"Unknown example_name '{example_name}'. "
                         "Supported examples: jacktoy, push_t, box_topple, ball_rolling")
    subprocess.run(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', logname])


if __name__ == '__main__':
    log_type = sys.argv[1]
    example_name = sys.argv[2]
    main(log_type, example_name)
