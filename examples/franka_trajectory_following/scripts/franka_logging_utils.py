import os
import glob
from datetime import date

def get_most_recent_logs():
  ''' change this to change log location '''
  log_root = "{}/adam_ws/logs".format(os.getenv('HOME'))

  curr_date = date.today().strftime("%m_%d_%y")
  year = date.today().strftime("%Y")
  logdir = "{}/{}/{}".format(log_root, year, curr_date)
  
  os.chdir(logdir)
  current_logs = sorted(glob.glob('*'))
  if len(current_logs) == 0:
    log_num = None
  elif current_logs[-1] == 'log_descriptions.txt':
    last_log = int(current_logs[-2])
    log_num = "{:02}".format(last_log)
  else:
    last_log = int(current_logs[-1])
    log_num = "{:02}".format(last_log)

  return logdir, log_num