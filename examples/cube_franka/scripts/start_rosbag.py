import subprocess
import os
import glob
import time
from franka_logging_utils import get_most_recent_logs

def main():
    # sleep for 0.1 seconds to ensure new log directory is finished being
    # created by start_logging.py
    time.sleep(0.1)
    logdir, log_num = get_most_recent_logs()
    if log_num is None:
        print("Did not find logs in {}".format(logdir))
        return
    
    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs:
        if current_logs[-1] == 'log_descriptions.txt':
            last_log = int(current_logs[-2])
        else:
            last_log = int(current_logs[-1])
        log_num = "{:02}".format(last_log)
    else:
        log_num = '00'

    os.chdir("{}/{}".format(logdir, log_num))
    subprocess.Popen(['rosbag', 'record', '-O', 'video.bag', '/camera/color/image_raw', '__name:=video_bag'])


if __name__ == '__main__':
    main()