import subprocess
import os
import codecs
import sys
from franka_logging_utils import *


def main(argv):
    computer = "c3"
    dair = "/home/sharanya/workspace/dairlib"
    logdir, log_num = create_new_log("/home/sharanya/workspace/dairlib/logs")

    git_diff = subprocess.check_output(['git', 'diff'], cwd=dair)
    commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dair)

    os.chdir('{}/{}'.format(logdir, log_num))
    
    with open('commit_tag%s' % log_num, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])
    
    parameters_path = "{}/examples/cube_franka/parameters.yaml".format(dair)
    subprocess.call(['cp', parameters_path, 'parameters{}.yaml'.format(log_num)])
  
    subprocess.call(['/opt/lcm/1.4.0/bin/lcm-logger', '-f', 'lcmlog-%s' % log_num])


if __name__ == '__main__':
    main(sys.argv)