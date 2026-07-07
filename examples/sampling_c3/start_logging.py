import subprocess
import os
import os.path as op
import glob
import codecs
from datetime import date
import sys
import yaml


def main(log_type, demo_name, folder_path):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{folder_path}/{year}/{curr_date}"
    dair = op.abspath(op.join(op.dirname(__file__), "../../"))

    os.makedirs(logdir, exist_ok=True)

    # Hardcoded sampling_c3_controller_params path.
    sampling_c3_controller_params_path = op.join(
        dair, "examples", "sampling_c3", demo_name, "parameters", 
        "sampling_c3_controller_params.yaml"
    )

    with open(sampling_c3_controller_params_path) as f:
        controller_params = yaml.load(f, Loader=yaml.FullLoader)

    sampling_c3_params_path = op.join(dair, controller_params['sampling_c3_options_file'])
    repos_params_path = op.join(dair, controller_params['reposition_params_file'])
    progress_params_path = op.join(dair, controller_params['progress_params_file'])
    sampling_params_path = op.join(dair, controller_params['sampling_params_file'])
    goal_params_path = op.join(dair, controller_params['goal_params_file'])
    sim_params_path = op.join(dair, controller_params['sim_params_file'])
    osc_params_path = op.join(dair, controller_params['osc_params_file'])
    osqp_params_path = op.join(dair, controller_params['osqp_settings_file'])
    osc_qp_params_path = op.join(dair, controller_params['osc_qp_settings_file'])

    with open(sim_params_path) as f:
        sim_params = yaml.load(f, Loader=yaml.FullLoader)

    # c3 gains, sampling c3 options, sampling, osc, sim, goal

    object_c3_urdf = op.join(dair, controller_params['object_models'][0])
    object_sim_urdf = op.join(dair, sim_params['object_models'][0])

    # NOTE:  must match kEndEffectorSimpleModel in sampling_c3_utils.h
    ee_simple_model_urdf = op.join(
        dair, "examples/sampling_c3/urdf/end_effector_simple_model.urdf")

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)

    try:
        directories = glob.glob(op.join(logdir, "*"))
        directory_names = [op.basename(d) for d in directories if op.isdir(d)]
        last_log = max([int(name) for name in directory_names if name.isdigit()])
        log_num = str(last_log+1).zfill(6)
    except:
        log_num = str(0).zfill(6)

    if log_type == 'hw':
        with open('commit_tag%s' % log_num, 'w') as f:
            f.write(str(commit_tag))
            f.write("\n\ngit diff:\n\n")
            f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
    if not op.isdir(log_num):
        os.mkdir(log_num)
    
    os.chdir(log_num)
    logname = f'{log_type}log-{log_num}'

    # Parameter files.
    subprocess.run(['cp',sampling_c3_controller_params_path, f'sampling_c3_controller_params_{log_num}.yaml'])
    subprocess.run(['cp',sampling_c3_params_path, f'sampling_c3_params_{log_num}.yaml'])
    subprocess.run(['cp',repos_params_path, f'repos_params_{log_num}.yaml'])
    subprocess.run(['cp',progress_params_path, f'progress_params_{log_num}.yaml'])
    subprocess.run(['cp',sampling_params_path, f'sampling_params_{log_num}.yaml'])
    subprocess.run(['cp',goal_params_path, f'goal_params_{log_num}.yaml'])
    subprocess.run(['cp',sim_params_path, f'sim_params_{log_num}.yaml'])
    subprocess.run(['cp',osc_params_path, f'osc_params_{log_num}.yaml'])
    subprocess.run(['cp',osqp_params_path, f'osqp_params_{log_num}.yaml'])
    subprocess.run(['cp',osc_qp_params_path, f'osc_qp_params_{log_num}.yaml'])

    # URDFs/SDFs with original file extensions.
    ee_simple_model_ext = op.splitext(ee_simple_model_urdf)[1]
    object_c3_ext = op.splitext(object_c3_urdf)[1]
    object_sim_ext = op.splitext(object_sim_urdf)[1]
    subprocess.run(['cp', ee_simple_model_urdf, f'ee_simple_model_urdf_{log_num}{ee_simple_model_ext}'])
    subprocess.run(['cp', object_c3_urdf, f'object_c3_urdf_{log_num}{object_c3_ext}'])
    subprocess.run(['cp', object_sim_urdf, f'object_sim_urdf_{log_num}{object_sim_ext}'])

    # Begin logging.
    subprocess.run(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', logname])


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(f"Usage: python {sys.argv[0]} <log_type> <demo_name> <folder_path>")
        sys.exit(1)
    log_type = sys.argv[1]
    demo_name = sys.argv[2]
    folder_path = sys.argv[3]
    main(log_type, demo_name, folder_path)
