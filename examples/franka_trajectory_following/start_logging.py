import subprocess
import os
import glob
import codecs
from datetime import date


def main():

    # Make the log folder if one doesn't already exist 
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/adam_ws/logs/{year}/{curr_date}"
    dair = f"{os.getenv('DAIR_PATH')}"

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

    subprocess.run(['lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    main()