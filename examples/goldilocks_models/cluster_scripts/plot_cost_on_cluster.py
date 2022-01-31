import subprocess
import time

# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

while True:
  # Call plotting script with srun
  RunCommand("srun --qos=low --nodes=1 --ntasks=1 --cpus-per-task=1 --mem-per-cpu=2G --pty python3 examples/goldilocks_models/find_models/plot_cost.py", True)

  print("sleep for a while")
  time.sleep(1800)

