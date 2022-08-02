import subprocess
import os
import glob
import codecs
from datetime import date


def main():

    # Make the log folder if one doesn't already exist 
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)
    dair = str(os.getenv('DAIR_PATH'))

    if not os.path.isdir(logdir):
        os.makedirs(logdir)

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs:
        if current_logs[-1] == 'log_descriptions.txt':
            last_log = int(current_logs[-2])
        else:
            last_log = int(current_logs[-1])
        log_num = "{:02}".format(last_log+1)
    else:
        log_num = '00'
    
    os.mkdir(log_num)
    os.chdir(log_num)
    with open('commit_tag%s' % log_num, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
    
    parameters_path = "{}/examples/franka_trajectory_following/parameters.yaml".format(dair)
    subprocess.Popen(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])
  
    subprocess.Popen(['lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    main()