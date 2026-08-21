import subprocess
import os
import glob
import codecs
from datetime import date
import sys

def main(log_type, folder_path=None):
    if folder_path:
        logdir = os.path.abspath(folder_path)
    else:
        curr_date = date.today().strftime("%m_%d_%y")
        year = date.today().strftime("%Y")
        logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"

    os.makedirs(logdir, exist_ok=True)

    dair = f"{os.getenv('HOME')}/dairlib/"

    print(f"Logging directory: {logdir}")

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)
    current_logs = sorted(
        f for f in glob.glob(f"{log_type}log-*")
        if not f.endswith(".jlp")
    )
    
    print(current_logs)
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

    excluded_channels = [
        "PMD_INFO2",
        "PMD_ORDERS2",
        "PMD_PRINTF",
        "iC3_LQR",
        "iC3_TRAJECTORY_LAMBDA",
        "iC3_TRAJECTORY_U",
        "iC3_TRAJECTORY_X",
    ]
    channel_regex = "^(" + "|".join(excluded_channels) + ")$"

    subprocess.run([
        'lcm-logger',
        '-f',
        '-v',
        '-c', channel_regex,
        log_type + 'log-%s' % log_num
    ])


if __name__ == '__main__':
    log_type = sys.argv[1]
    folder_path = sys.argv[2] if len(sys.argv) > 2 else None
    main(log_type, folder_path)
