# Note: Run this script AFTER the realsense camera has been turned on
# and after start logging

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import glob
from datetime import date
import subprocess
import time

class Recorder:
  def __init__(self):
    ''' ROS subscribers '''
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
    self.frame = 0

    ''' directory paths '''
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    self.logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)
    self.dair = str(os.getenv('DAIR_PATH'))

    ''' locate most recent log '''
    os.chdir(self.logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs[-1] == 'log_descriptions.txt':
      last_log = int(current_logs[-2])
    else:
      last_log = int(current_logs[-1])
    self.log_num = "{:02}".format(last_log)

    ''' remove existing frame dir if applicable and create new frames dir '''
    self.frames_dir = "{}/{}/frames".format(self.logdir, self.log_num)
    subprocess.Popen(['rm', '-rf', self.frames_dir])
    subprocess.Popen(['mkdir', self.frames_dir])
    self.prev_time = time.time()

  def callback(self, msg):
    start = time.time()
    try:
      img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)
    
    frame_num = "frame{:04}".format(self.frame)
    cv2.imwrite("{}/{}.png".format(self.frames_dir, frame_num), 
      cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    self.frame += 1
    print(time.time()-start)

    # now = time.time()
    # print(now - self.prev_time)
    # self.prev_time = now

def main():
  rospy.init_node("start_filming")
  Recorder()
  rospy.spin()

if __name__ == "__main__":
  main()