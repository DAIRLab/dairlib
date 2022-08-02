import subprocess
import os
import glob
import codecs
from datetime import date
import time

def main():

    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)
    dair = str(os.getenv('DAIR_PATH'))

    # sleep for 0.2 seconds to ensure new log directory is finished being
    # created by start_logging
    time.sleep(0.2)
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