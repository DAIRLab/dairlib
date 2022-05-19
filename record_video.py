import subprocess
import sys
import os
import glob
from datetime import date
def main():
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/Videos/cassie_experiments/{year}/{curr_date}/yuming_rom_walking/"

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    current_logs = sorted(glob.glob(logdir + 'log_*'))
    if current_logs:
        last_log = int(current_logs[-1].split('_')[-1].split('.')[0])
        log_num = f'{last_log+1:02}'
    else:
        log_num = '00'
    print(log_num)
    print(current_logs)
    experiment_name = logdir + 'log_' + log_num + '.mp4'
    cmd = 'v4l2-ctl --list-devices'.split(' ')
    process = subprocess.check_output(cmd)
    lines = process.decode("utf-8").split('\n')
    dev_name = ''
    for i, line in enumerate(lines):
        if 'HD' in line:
            dev_name = lines[i+1].split('\t')[1]
    print(dev_name)
    cmds = ('ffmpeg -y -f alsa -i default -i ' + dev_name + ' -vcodec libx264 -acodec aac -qp 0').split(' ')
    cmds.append(experiment_name)
    print(cmds)
    # import pdb; pdb.set_trace()
    subprocess.run(cmds)

if __name__ == '__main__':
    main()
