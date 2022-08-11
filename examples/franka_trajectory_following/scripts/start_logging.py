import subprocess
import os
import codecs
import sys
from franka_logging_utils import get_most_recent_logs


def main(argv):
    dair = str(os.getenv('DAIR_PATH'))
    logdir, log_num = get_most_recent_logs()

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    if not os.path.isdir(logdir):
        os.makedirs(logdir)
    os.chdir(logdir)
    
    if log_num is None:
      log_num = '00'
    else:
      log_num = "{:02}".format(int(log_num)+1)
    
    os.mkdir(log_num)
    os.chdir(log_num)
    if len(argv) == 1: # log on franka computer
      with open('commit_tag%s' % log_num, 'w') as f:
          f.write(str(commit_tag))
          f.write("\n\ngit diff:\n\n")
          f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
      
      parameters_path = "{}/examples/franka_trajectory_following/parameters.yaml".format(dair)
      subprocess.call(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])
    
      subprocess.call(['lcm-logger', '-f', 'lcmlog-%s' % log_num])
    elif len(argv) == 2: # log on c3 computer
      parameters_path = "{}/examples/franka_trajectory_following/parameters.yaml".format(dair)
      subprocess.call(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])


if __name__ == '__main__':
    main(sys.argv)