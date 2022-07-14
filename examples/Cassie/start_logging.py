import subprocess
import os
import glob
import codecs
from datetime import date


def main():
    sim = (os.getlogin() == 'brian')
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")

    logdir = \
        f"{os.getenv('HOME')}/workspace/logs/cassie_simulation/{year}/{curr_date}" \
            if sim else f"{os.getenv('HOME')}/logs/{year}/{curr_date}"

    current_dair_dir = f"{os.getcwd()}/"
    standing_gains = current_dair_dir + "examples/Cassie/osc/osc_standing_gains.yaml"
    walking_gains = current_dair_dir + "examples/Cassie/osc/osc_walking_gains.yaml"
    alip_gains = current_dair_dir + "examples/Cassie/osc/osc_walking_gains_alip.yaml"
    running_gains = current_dair_dir + "examples/Cassie/osc_run/osc_running_gains.yaml"

    if not os.path.isdir(logdir):
        os.mkdir(logdir)

    git_diff = subprocess.check_output(['git', 'diff'], cwd=current_dair_dir)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=current_dair_dir)

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

    subprocess.run(['cp', standing_gains, 'standing_gains_%s.yaml' % log_num])
    subprocess.run(['cp', walking_gains, 'walking_gains_%s.yaml' % log_num])
    subprocess.run(['cp', alip_gains, 'walking_gains_alip%s.yaml' % log_num])
    subprocess.run(['cp', running_gains, 'running_gains_%s.yaml' % log_num])
    subprocess.run(['lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    main()
