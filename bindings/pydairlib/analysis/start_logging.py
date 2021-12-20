import subprocess
import sys
import os
import glob
import codecs
from datetime import date

if __name__ == '__main__':

    mpc_controller = sys.argv[1]

    log_folder_map = {'srbd': 'cmpc_debug',
                      'residual': 'cmpc_debug_residual',
                      'osc': 'osc_walking'}

    curr_date = date.today().strftime("%m_%d_%y")

    logdir = f"{os.getenv('HOME')}/workspace/" \
             f"logs/{log_folder_map[mpc_controller]}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/dairlib/"
    mpc_gains_file = dair + "examples/Cassie/mpc/cassie_srbd_cmpc_gains.yaml"
    osc_gains_file = dair + "examples/Cassie/mpc/" \
                            "cassie_mpc_osc_walking_gains.yaml"

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)
    current_logs = sorted(glob.glob('lcmlog-*'))
    if current_logs:
        last_log = int(current_logs[-1].split('-')[-1])
        log_num = f'{last_log+1:02}'
    else:
        log_num = '00'

    with open('commit_tag%s' % log_num, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])

    subprocess.run(['cp', mpc_gains_file, 'mpc_gains_%s.yaml' % log_num])
    subprocess.run(['cp', osc_gains_file, 'osc_gains_%s.yaml' % log_num])
    subprocess.run(['lcm-logger', '-f', 'lcmlog-%s' % log_num])
