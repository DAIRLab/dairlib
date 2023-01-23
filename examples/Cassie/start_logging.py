import subprocess
import os
import sys
import glob
import codecs
import socket
from datetime import date


def is_cassie():
    return socket.gethostname() == 'dair-cassie'


def mpc_debug_logging_main():
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/brian/dairlib/"

    mpc_gains = os.path.join(
        dair,
        "examples/perceptive_locomotion/gains/alip_minlp_gains.yaml"
    )

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    os.chdir(logdir)
    current_logs = sorted(glob.glob('lcmlog-mpc-*'))
    if current_logs:
        last_log = int(current_logs[-1].split('-')[-1])
        log_num = f'{last_log+1:02}'
    else:
        log_num = '00'

    with open('commit_tag-%s' % log_num, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])

    subprocess.run(['cp', mpc_gains, 'alip_minlp_gains_%s.yaml' % log_num])
    subprocess.run(['lcm-logger', '-f', 'lcmlog-mpc-%s' % log_num])


def log_everything_main():
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/{'brian/' if is_cassie() else''}" \
           f"dairlib/"
    standing_gains = os.path.join(
        dair,
        "examples/Cassie/osc/osc_standing_gains.yaml"
    )
    mpc_gains = os.path.join(
        dair,
        "examples/perceptive_locomotion/gains/alip_minlp_gains.yaml"
    )
    osc_gains = os.path.join(
        dair,
        "examples/perceptive_locomotion/gains/osc_gains.yaml"
    )

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

    with open('commit_tag-%s' % log_num, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])

    subprocess.run(['cp', standing_gains, 'standing_gains_%s.yaml' % log_num])
    subprocess.run(['cp', mpc_gains, 'alip_minlp_gains_%s.yaml' % log_num])
    subprocess.run(['cp', osc_gains, 'alip_minlp_osc_gains_%s.yaml' % log_num])
    subprocess.run(['lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == 'cassie':
            log_everything_main()
        elif sys.argv[1] == 'cassie-laptop':
            mpc_debug_logging_main()
        else:
            print("invalid computer - quitting")
    else:
        log_everything_main()
