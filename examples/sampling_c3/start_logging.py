import subprocess
import os
import os.path as op
import glob
import codecs
import shutil
from datetime import date
import sys
import yaml


UBUNTU_22_LCM_COMMAND = '/opt/lcm/1.4.0/bin/lcm-logger'
UBUNTU_24_LCM_COMMAND = 'lcm-logger'
LCM_COMMAND = UBUNTU_24_LCM_COMMAND  # Change this if using Ubuntu 22.

# Parameter files referenced by sampling_c3_controller_params.yaml, as
# (parameter key, logged file prefix, required) tuples.  Required entries warn
# if the key is missing or points at a nonexistent file; optional entries are
# only used by some demos (e.g. risk_params_file) or are explicitly marked
# UNUSED.
PARAM_FILES = [
    ('sampling_c3_options_file', 'sampling_c3_params', True),
    ('reposition_params_file', 'repos_params', True),
    ('progress_params_file', 'progress_params', True),
    ('sampling_params_file', 'sampling_params', True),
    ('goal_params_file', 'goal_params', True),
    ('sim_params_file', 'sim_params', True),
    ('vis_params_file', 'vis_params', True),
    ('osc_params_file', 'osc_params', True),
    ('osqp_settings_file', 'osqp_params', True),
    ('osc_qp_settings_file', 'osc_qp_params', True),
    ('lcm_channels_hardware_file', 'lcm_channels_hardware', True),
    ('lcm_channels_simulation_file', 'lcm_channels_simulation', True),
    ('franka_driver_channels_file', 'franka_driver_channels', False),
    ('risk_params_file', 'risk_params', False),
]

# NOTE:  must match kEndEffectorSimpleModel in sampling_c3_utils.h
EE_SIMPLE_MODEL_URDF = 'examples/sampling_c3/urdf/end_effector_simple_model.urdf'


def resolve(dair, params, key, required):
    """Returns the absolute path referenced by params[key], or None if the key
    is unset or explicitly UNUSED."""
    rel_path = params.get(key)
    if rel_path is None or rel_path == 'UNUSED':
        if required:
            print(f'WARNING:  {key} is not set; not logging it.')
        return None
    return op.join(dair, rel_path)


def copy_into_log(src, dest):
    """Copies src into the current log directory as dest, warning instead of
    failing silently if src does not exist."""
    if src is None:
        return
    if not op.isfile(src):
        print(f'WARNING:  Could not log {dest}; no such file {src}')
        return
    shutil.copy(src, dest)


def copy_models_into_log(dair, rel_paths, prefix, log_num):
    """Copies a list of model files into the current log directory, keeping
    their original file extensions.  Indices are appended only when there is
    more than one model, so single-object demos keep their historical names."""
    rel_paths = [p for p in (rel_paths or []) if p]
    for i, rel_path in enumerate(rel_paths):
        ext = op.splitext(rel_path)[1]
        suffix = f'_{i}' if len(rel_paths) > 1 else ''
        copy_into_log(op.join(dair, rel_path), f'{prefix}_{log_num}{suffix}{ext}')


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

    param_file_paths = {
        key: resolve(dair, controller_params, key, required)
        for key, _, required in PARAM_FILES
    }

    with open(param_file_paths['sim_params_file']) as f:
        sim_params = yaml.load(f, Loader=yaml.FullLoader)

    git_diff = subprocess.check_output(['git', 'diff', 'HEAD'], cwd=dair)
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
    copy_into_log(sampling_c3_controller_params_path,
                  f'sampling_c3_controller_params_{log_num}.yaml')
    for key, prefix, _ in PARAM_FILES:
        copy_into_log(param_file_paths[key], f'{prefix}_{log_num}.yaml')

    # URDFs/SDFs with original file extensions.
    copy_models_into_log(dair, [EE_SIMPLE_MODEL_URDF], 'ee_simple_model_urdf',
                         log_num)
    copy_models_into_log(dair, controller_params['object_models'],
                         'object_c3_urdf', log_num)
    copy_models_into_log(dair, sim_params['object_models'], 'object_sim_urdf',
                         log_num)
    copy_models_into_log(dair, controller_params.get('keep_out_model_sequence'),
                         'keep_out_urdf', log_num)

    # Begin logging.
    subprocess.run([LCM_COMMAND, '-f', logname])


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(f"Usage: python {sys.argv[0]} <log_type> <demo_name> <folder_path>")
        sys.exit(1)
    log_type = sys.argv[1]
    demo_name = sys.argv[2]
    folder_path = sys.argv[3]
    main(log_type, demo_name, folder_path)
