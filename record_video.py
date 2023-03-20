import subprocess
import sys
import os
import glob
import signal

from datetime import date
def main():
    # get devices
    dev_names = []
    cmd = 'v4l2-ctl --list-devices'.split(' ')
    process = ''
    try:
        process = subprocess.check_output(cmd)
    except subprocess.SubprocessError as err:
        process = err.output
        pass

    lines = process.decode("utf-8").split('\n')
    for i, line in enumerate(lines):
        if 'HD' in line:
            dev_names.append(lines[i+1].split('\t')[1])
    print(dev_names)
    cmds = []

    # determine log number
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/Videos/cassie_experiments/{year}/{curr_date}/"
    os.makedirs(logdir, exist_ok=True)
    current_logs = sorted(glob.glob(logdir + '*log_*'))
    if current_logs:
        last_log = int(current_logs[-1].split('_')[-2].split('.')[0])
        log_num = f'{last_log+1:02}'
    else:
        log_num = '00'

    # construct video writing commands
    for i, name in enumerate(dev_names):
        experiment_name = logdir + 'log_' + log_num + '_cam' + str(i) + '.mp4'
        # print(experiment_name)
        cmd = ('ffmpeg -y -f alsa -i default -i ' + name + ' -vcodec libx264 -acodec aac -qp 0').split(' ')
        cmd.append(experiment_name)
        print(' '.join(cmd))
        cmds.append(cmd)
    # import pdb; pdb.set_trace()
    processes = []
    for cmd in cmds:
        processes.append(subprocess.Popen(cmd))
    for process in processes:
        process.communicate()


if __name__ == '__main__':
    main()
