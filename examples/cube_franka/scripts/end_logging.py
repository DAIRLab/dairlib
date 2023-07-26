import os
import sys
import select
from franka_logging_utils import get_most_recent_logs

def main():
    logdir, log_num = get_most_recent_logs()
    if log_num is None:
        print("Did not find logs in {}".format(logdir))
        return

    os.chdir(logdir)
    description = ''
    with open('log_descriptions.txt', 'a') as f:
        f.write(log_num + ": ")

        # wait 2 min for user to input description
        print("Provide a short description of this log. Press [Enter] to continue: ")
        i, o, e = select.select( [sys.stdin], [], [], 120)
        if i:
          description = sys.stdin.readline().strip()
          f.write(description)
          print("Description logged.")
        else:
          print("Timeout. A description can still be added by editing the log files directly.")
        f.write('\n')

    with open('{}/log_description{}.txt'.format(log_num, log_num), 'a') as f:
        f.write(log_num + ": ")
        f.write(description)
        f.write('\n')


if __name__ == '__main__':
    main()