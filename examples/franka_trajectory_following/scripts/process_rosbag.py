import subprocess
import os
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

    os.chdir("{}/{}".format(logdir, log_num))
    subprocess.Popen(['roslaunch', 'franka_vision', 'export_images.launch', 'bag_path:={}/{}/video.bag'.format(logdir, log_num)])

if __name__ == '__main__':
    main()