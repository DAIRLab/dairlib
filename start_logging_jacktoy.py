import subprocess
import os
import os.path as op
import glob
import codecs
from datetime import date
import sys
import yaml

def main(log_type):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    dair = op.dirname(os.path.abspath(__file__))
    if log_type == 'mjpc':
        logdir = f'/mnt/data2/bibit/logs/{year}/{curr_date}'
    else:
        logdir = f"/mnt/data2/sharanya/logs/{year}/{curr_date}"

    if not op.isdir(logdir):
        os.mkdir(logdir)

    # franka_cr_controller params path
    franka_c3_controller_params_path = op.join(
        dair, "examples/jacktoy/parameters/franka_c3_controller_params.yaml")

    # Load the run_in_safe_mode param from dair + "examples/jacktoy/parameters/franka_c3_controller_params.yaml"
    with open(franka_c3_controller_params_path) as f:
        franka_c3_controller_params = yaml.load(f, Loader=yaml.FullLoader)

    # if franka_c3_controller_params.yaml has run_in_safe_mode: true, then load the safe mode gains
    if (franka_c3_controller_params['run_in_safe_mode']):
        c3_gains = op.join(
            dair, "examples/jacktoy/parameters/franka_c3_options_floating_safe.yaml")
        sampling_params = op.join(
            dair, "examples/jacktoy/parameters/sampling_params_safe.yaml")
    else:
        c3_gains = op.join(
            dair, "examples/jacktoy/parameters/franka_c3_options_floating.yaml")
        sampling_params = op.join(
            dair, "examples/jacktoy/parameters/sampling_params.yaml")

    osc_gains = op.join(
        dair, "examples/jacktoy/parameters/franka_osc_controller_params.yaml")
    sim_params = op.join(
        dair, "examples/jacktoy/parameters/franka_sim_params.yaml")
    trajectory_params = op.join(
        dair, "examples/jacktoy/parameters/trajectory_params.yaml")
    ee_simple_model_urdf = op.join(
        dair, "examples/jacktoy/urdf/end_effector_simple_model.urdf")
    jack_sdf = op.join(
        dair, "examples/jacktoy/urdf/jack.sdf")

    if log_type == 'mjpc':
        mjpc_jack = op.join(op.dirname(dair), 'mujoco_mpc', 'mjpc', 'tasks', 'jack')
        mjpc_task = op.join(mjpc_jack, 'task.xml')
        mjpc_ee = op.join(mjpc_jack, 'end_effector.xml')

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
    subprocess.run(['cp', franka_c3_controller_params_path, f'franka_c3_controller_params_{log_num}.yaml'])
    subprocess.run(['cp', osc_gains, f'osc_gains_{log_num}.yaml'])
    subprocess.run(['cp', sim_params, f'sim_params_{log_num}.yaml'])
    subprocess.run(['cp', c3_gains, f'c3_gains_{log_num}.yaml'])
    subprocess.run(['cp', sampling_params, f'sampling_params_{log_num}.yaml'])
    subprocess.run(['cp', trajectory_params, f'trajectory_params_{log_num}.yaml'])
    subprocess.run(['cp', ee_simple_model_urdf, f'ee_simple_model_urdf_{log_num}.urdf'])
    subprocess.run(['cp', jack_sdf, f'jack_sdf_{log_num}.sdf'])
    if log_type == 'mjpc':
        subprocess.run(['cp', mjpc_task, f'mjpc_task_{log_num}.xml'])
        subprocess.run(['cp', mjpc_ee, f'mjpc_end_effector_{log_num}.xml'])

    subprocess.run(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', logname])


if __name__ == '__main__':
    log_type = sys.argv[1]
    main(log_type)
