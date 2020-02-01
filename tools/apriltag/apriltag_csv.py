import rosbag
import subprocess, yaml
import numpy as np
import pdb
import sys

from scipy import interpolate

# static parameters
CUBE_TOPIC_STRING = '/tagslam/odom/body_cube'
BOARD_TOPIC_STRING = '/tagslam/odom/body_surface'

if len(sys.argv) < 3:
    print("usage: python[2] apriltag_csv.py [ROSBAG] [CSV_OUT]")
    sys.exit()

inbag = sys.argv[1]
outcsv = sys.argv[2]

# open rosbag
bag = rosbag.Bag(inbag)

# get summary info from rosbag as a dictionary
info_dict = yaml.load(bag._get_yaml_info())

# extract metadata from cube and board topics
cube_topic = [topic for topic in info_dict['topics'] if topic['topic'] == CUBE_TOPIC_STRING][0]
board_topic = [topic for topic in info_dict['topics'] if topic['topic'] == BOARD_TOPIC_STRING][0]

# ensure there are an equal number of cube and board odom messages
num_msg = cube_topic['messages']
if not num_msg == board_topic['messages']:
    raise Exception('Missing odom messages for board and/or cube!')

# extract cube pose data
t_ros = np.zeros(num_msg)
cube_ros = np.zeros((7,num_msg))
i = 0
for i, tmt in enumerate(bag.read_messages(topics=[CUBE_TOPIC_STRING])):
    topic, msg, t = tmt
    tstamp = msg.header.stamp
    t_ros[i] = tstamp.secs + tstamp.nsecs*1e-9
    cube_pose = msg.pose.pose
    cube_pos = np.asarray([cube_pose.position.x, cube_pose.position.y, cube_pose.position.z])
    cube_quat = np.asarray([cube_pose.orientation.x, cube_pose.orientation.y, cube_pose.orientation.z, cube_pose.orientation.w])
    cube_ros[:4,i] = cube_quat
    cube_ros[4:7,i] = cube_pos
    i = i + 1

# extract board pose data
board_ros = np.zeros((7,num_msg))
i = 0
for i, tmt in enumerate(bag.read_messages(topics=[BOARD_TOPIC_STRING])):
    topic, msg, t = tmt
    board_pose = msg.pose.pose
    board_pos = np.asarray([board_pose.position.x, board_pose.position.y, board_pose.position.z])
    board_quat = np.asarray([board_pose.orientation.x, board_pose.orientation.y, board_pose.orientation.z, board_pose.orientation.w])
    board_ros[:4,i] = board_quat
    board_ros[4:7,i] = board_pos
    i = i + 1
bag.close()

# assemble CSV data and save
data = np.concatenate((np.expand_dims(t_ros, axis=0), cube_ros, board_ros), axis=0)
data = data.T
np.savetxt(outcsv, data, delimiter=',')
