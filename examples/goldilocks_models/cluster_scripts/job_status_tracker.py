# Run this script at the root of Dairlib directory

import subprocess
import time
import codecs
import os

# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

# https://stackoverflow.com/questions/2502833/store-output-of-subprocess-popen-call-in-a-string
def GetCommandOutput(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell, stdout=subprocess.PIPE)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

  output_bytes = process.communicate()[0]
  output_string = codecs.getdecoder("unicode_escape")(output_bytes)[0]
  return output_string


# Parameters
status_file_path = "../job_status.txt"
first_line = "JOBID, ST, command\n"

while True:
  # Read current status
  '''
  Example output of `squeue -u yminchen -o "%i %t"`
  JOBID ST
  148205 R
  148209 R
  148202 R
  148196 PD
  148008 R
  148016 R
  '''
  output = GetCommandOutput("squeue -u yminchen -o \"%i %t\"", True)
  parsed_output = output.split("\n")  # split by newline
  parsed_output = [e for e in parsed_output[1:] if len(e) > 0]  # ignore the first line and get rid of empty element
  parsed_output = [e.split(" ") for e in parsed_output]  # split again by SPACE

  # Append commands of each job
  for line in parsed_output:
    line.append(GetCommandOutput("scontrol show job -d %s | grep Command;" % line[0], True))

  # Remove plotting jobs
  parsed_output = [e for e in parsed_output if "Command=python3" not in e[2]]

  merged_output = []
  if os.path.exists(status_file_path):
    # Merge the current status with the old status
    f = open(status_file_path, "r")
    text = f.read()
    f.close()
    parsed_output_history = text.split("\n")  # split by newline
    parsed_output_history = [e for e in parsed_output_history[1:] if len(e) > 0]  # get rid of empty element
    parsed_output_history = [e.split(", ") for e in parsed_output_history]  # split again by COMMA

    # Add new job to file
    for line in parsed_output:
      current_job_id = line[0]

      job_exist_in_file = False
      for old_line in parsed_output_history:
        if current_job_id == old_line[0]:
          job_exist_in_file = True
          break

      if not job_exist_in_file:
        parsed_output_history.append(line)

    # Update old job status
    for old_line in parsed_output_history:
      job_id = old_line[0]

      job_exist_current = False
      for line in parsed_output:
        if job_id == line[0]:
          job_exist_current = True
          break

      if not job_exist_current:
        old_line[1] = "inactive"

    merged_output = parsed_output_history

  else:
    merged_output = parsed_output

  # Clean up data -- remove leading white spaces
  merged_output = [[e.lstrip() for e in line] for line in merged_output]

  # Write jobs into file
  f = open(status_file_path, "w")
  f.write(first_line)
  f.write("\n".join([", ".join(line) for line in merged_output]))
  f.write("\n")
  f.close()

  # break
  time.sleep(600)



