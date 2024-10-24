import subprocess
import os
import glob
import codecs
from datetime import date


def main():
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"
    dair = f"{os.getenv('HOME')}/workspace/dairlib/"
    standing_gains = dair + "examples/Cassie/osc/osc_standing_gains.yaml"
    walking_gains = dair + "examples/Cassie/osc/osc_walking_gains.yaml"
    alip_walking_gains = dair + "examples/Cassie/osc/alip_osc_walking_gains.yaml"

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

    subprocess.run(['cp', standing_gains, 'standing_gains_%s.yaml' % log_num])
    subprocess.run(['cp', walking_gains, 'walking_gains_%s.yaml' % log_num])
    subprocess.run(['cp', alip_walking_gains,
                    'walking_gains_alip_%s.yaml' % log_num])
    subprocess.run(['lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    main()
