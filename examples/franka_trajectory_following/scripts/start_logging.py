import subprocess
import os
import codecs
import sys
from franka_logging_utils import *


def main(argv):
    if len(argv) == 1: # log on franka_computer
      computer = "franka"
      dair = str(os.getenv('DAIR_PATH'))
      logdir, log_num = create_new_log()
    elif len(argv) == 2:
      computer = "c3"
      dair = "/home/sharanya/workspace/dairlib"
      logdir, log_num = create_new_log("/home/sharanya/workspace")

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir('{}/{}'.format(logdir, log_num))
    if computer == "franka": # log on franka computer
      with open('commit_tag%s' % log_num, 'w') as f:
          f.write(str(commit_tag))
          f.write("\n\ngit diff:\n\n")
          f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
      
      parameters_path = "{}/examples/franka_trajectory_following/parameters.yaml".format(dair)
      subprocess.call(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])
    
      subprocess.call(['lcm-logger', '-f', 'lcmlog-%s' % log_num])
    elif computer == "c3": # log on c3 computer
      parameters_path = "{}/examples/franka_trajectory_following/parameters.yaml".format(dair)
      subprocess.call(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])


if __name__ == '__main__':
    main(sys.argv)