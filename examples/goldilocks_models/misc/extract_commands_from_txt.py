# This file does
# 1. extract commands from x_x_commands.txt files, and save them into .csv files (e.g. 1_30_des_stride_length.csv)

import numpy as np
import argparse
import os
import shutil

# Look for the first "%" of the line (excluding \%)
# If there doesn't exist a comment, it return the length of the string
def find_index_of_first_comment_in_a_line(line_string):
  assert line_string.count("\n") <= 1
  for i in range(len(line)):
    if line[i] == "%":
      if i == 0:
        return i
      else:
        if line[i-1] == "\\":
          continue
        else:
          return i
  return len(line)


### argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--eval_folder_path", help="/home/yuming/workspace/dairlib_data/goldilocks_models/sim_cost_eval/", default="", type=str)
parser.add_argument("--rom_max_iter", help="", default=1000, type=int)
parser.add_argument("--rom_max_log_idx", help="", default=1000, type=int)
args = parser.parse_args()
assert os.path.exists(args.eval_folder_path) and os.path.isdir(args.eval_folder_path)
assert args.eval_folder_path[-1] == "/"

task_name_list_to_save = ["stride_length","pelvis_height","ground_incline","turning_rate"]
for task_name in task_name_list_to_save:
  for rom_iter in range(args.rom_max_iter):
    for log_idx in range(args.rom_max_log_idx):
      command_txt_path = "%s%d_%d_commands.txt" % (args.eval_folder_path, rom_iter, log_idx)
      if os.path.exists(command_txt_path):
        command_value = ""

        f = open(command_txt_path,'r')
        for line in f.readlines():
          idx_of_substring = line.find("--%s=" % task_name)
          if idx_of_substring >= 0:
            idx_of_space_character = idx_of_substring
            for idx in range(idx_of_substring, len(line)):
              if line[idx] == " ":
                idx_of_space_character = idx
                break
            command_value = line[idx_of_substring:idx_of_space_character]
            break
        f.close()
        assert len(command_value) > 0

        f = open(args.eval_folder_path + '%d_%d_des_%s.csv' % (rom_iter, log_idx, task_name), "w")
        f.write(command_value)
        f.close()          

print("done")
