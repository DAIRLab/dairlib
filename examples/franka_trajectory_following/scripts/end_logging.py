import subprocess
import os
import glob
import codecs
import sys
import select
from datetime import date


def main():

    # Make the log folder if one doesn't already exist 
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)
    
    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs[-1] == 'log_descriptions.txt':
      last_log = int(current_logs[-2])
    else:
      last_log = int(current_logs[-1])
    log_num = "{:02}".format(last_log)

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