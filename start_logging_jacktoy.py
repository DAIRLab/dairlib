import subprocess
import os
import glob
import codecs
from datetime import date
import sys
import yaml

def main(log_type):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/dairlib/"

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    # franka_cr_controller params path
    franka_c3_controller_params_path = dair + "examples/jacktoy/parameters/franka_c3_controller_params.yaml"

    # Load the run_in_safe_mode param from dair + "examples/jacktoy/parameters/franka_c3_controller_params.yaml"
    with open(franka_c3_controller_params_path) as f:
        franka_c3_controller_params = yaml.load(f, Loader=yaml.FullLoader)

    # if franka_c3_controller_params.yaml has run_in_safe_mode: true, then load the safe mode gains
    if (franka_c3_controller_params['run_in_safe_mode']):
        c3_gains = dair + "examples/jacktoy/parameters/franka_c3_options_floating_safe.yaml"
        sampling_params = dair + "examples/jacktoy/parameters/sampling_params_safe.yaml"
    else:
        c3_gains = dair + "examples/jacktoy/parameters/franka_c3_options_floating.yaml"
        sampling_params = dair + "examples/jacktoy/parameters/sampling_params.yaml"

    osc_gains = dair + "examples/jacktoy/parameters/franka_osc_controller_params.yaml"
    sim_params = dair + "examples/jacktoy/parameters/franka_sim_params.yaml"
    trajectory_params = dair + "examples/jacktoy/parameters/trajectory_params.yaml"
    ee_simple_model_urdf = dair + "examples/jacktoy/urdf/end_effector_simple_model.urdf"
    jack_sdf = dair + "examples/jacktoy/urdf/jack.sdf"

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
    subprocess.run(['cp', sampling_params, f'sampling_params_{log_num}.yaml'])
    subprocess.run(['cp', trajectory_params, f'trajectory_params_{log_num}.yaml'])
    subprocess.run(['cp', ee_simple_model_urdf, f'ee_simple_model_urdf_{log_num}.urdf'])
    subprocess.run(['cp', jack_sdf, f'jack_sdf_{log_num}.sdf'])
    subprocess.run(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', logname])


if __name__ == '__main__':
    log_type = sys.argv[1]
    main(log_type)
