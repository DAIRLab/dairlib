import subprocess
import os
import time
from datetime import datetime

# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

### argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--idx", help="", default=0, type=int)
parser.add_argument("--ip", help="", default="192.168.1.140", type=str)
# parser.add_argument("--path", help="", default="", type=str)
args = parser.parse_args()

file_names = ["rom_trajectory", "x_init.csv"]

RunCommand("scp dair@%s:/home/dair/workspace/yuming/dairlib_data/goldilocks_models/planning/robot_1/data/%d_* ../dairlib_data/goldilocks_models/planning/robot_1/data/." % (args.ip, args.idx), True)
for name in file_names:
  RunCommand("mv ../dairlib_data/goldilocks_models/planning/robot_1/data/%d_%s ../dairlib_data/goldilocks_models/planning/robot_1/data/0_%s" % (args.idx, name, name), True)

print("end of script.")
