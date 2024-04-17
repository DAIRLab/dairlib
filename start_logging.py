import subprocess
import os
import glob
import codecs
from datetime import date
import sys

def main(log_type):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/dairlib/"

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    osc_gains = dair + "examples/jacktoy/parameters/franka_osc_controller_params.yaml"
    sim_params = dair + "examples/jacktoy/parameters/franka_sim_params.yaml"
    c3_gains = dair + "examples/jacktoy/parameters/franka_c3_options_floating.yaml"
    sampling_params = dair + "examples/jacktoy/parameters/sampling_params.yaml"
    trajectory_params = dair + "examples/jacktoy/parameters/trajectory_params.yaml"

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    try:
        last_log = int(current_logs[-1].split('-')[-1])
        log_num = f'{last_log+1:02}'
    except:
        log_num = '00'

    if log_type == 'hw':
        with open('commit_tag%s' % log_num, 'w') as f:
            f.write(str(commit_tag))
            f.write("\n\ngit diff:\n\n")
            f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
    if not os.path.isdir(log_num):
        os.mkdir(log_num)
    
    os.chdir(log_num)
    logname = f'{log_type}log-{log_num}'
    subprocess.run(['cp', osc_gains, f'osc_gains_{log_num}.yaml'])
    subprocess.run(['cp', sim_params, f'sim_params_{log_num}.yaml'])
    subprocess.run(['cp', c3_gains, f'c3_gains_{log_num}.yaml'])
    subprocess.run(['cp', sampling_params, f'sampling_params_{log_num}.yaml'])
    subprocess.run(['cp', trajectory_params, f'trajectory_params_{log_num}.yaml'])
    subprocess.run(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', logname])


if __name__ == '__main__':
    log_type = sys.argv[1]
    main(log_type)
