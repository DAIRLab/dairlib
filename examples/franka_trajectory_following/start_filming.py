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

class Recorder:
  def __init__(sefl):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
    self.frame = 0

    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    self.logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)
    self.dair = str(os.getenv('DAIR_PATH'))

    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs[-1] == 'log_descriptions.txt':
      last_log = int(current_logs[-2])
    else:
      last_log = int(current_logs[-1])
    self.log_num = "{:02}".format(last_log)

    self.frames_dir = "{}/{}/frames".format(self.logdir, self.log_num)
    os.mkdir(self.frames_dir)

def callback(msg):
  try:
    img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
  except CvBridgeError as e:
    print(e)
  
  frame_num = "{:05}".format(self.frame)
  cv2.imwrite("{}/{}.png".format(self.frames_dir, frame_num), img)
  self.frame += 1

def main():
  Recorder()

if __name__ == "__main__":
  main()