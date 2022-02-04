# This script tracks every job's status and is able to re-submit jobs if specified.
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
nonstop_sbatch_script = []  # re-submit scripts if added. Only file name is required (don't need directory)

# Some setups
nonstop_sbatch_script = [e.split("/")[-1] for e in nonstop_sbatch_script]  # Get rid of directory. Only keep file name

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
  output = GetCommandOutput("squeue -u yminchen -o \"%i, %t, %o\"", True)
  parsed_output = output.split("\n")  # split by newline
  parsed_output = [e for e in parsed_output[1:] if len(e) > 0]  # ignore the first line and get rid of empty line
  parsed_output = [e.split(", ") for e in parsed_output]  # split again by ", "

  # Remove plotting jobs
  parsed_output = [e for e in parsed_output if "python3" not in e[2]]

  merged_output = []
  if os.path.exists(status_file_path):
    # Merge the current status with the old status
    f = open(status_file_path, "r")
    history_output = f.read()
    f.close()
    parsed_history_output = history_output.split("\n")  # split by newline
    parsed_history_output = [e for e in parsed_history_output[1:] if len(e) > 0]  # ignore the first line and get rid of empty line
    parsed_history_output = [e.split(", ") for e in parsed_history_output]  # split again by COMMA

    # Add new job to file
    for line in parsed_output:
      job_exist_in_file = line[0] in list(zip(*parsed_history_output))[0]  # check if the new job id exists in the file
      if not job_exist_in_file:
        parsed_history_output.append(line)

    # Update old job status
    for old_line in parsed_history_output:
      job_id = old_line[0]

      transpose_parsed_output = [list(x) for x in zip(*parsed_output)]
      job_currently_exists = job_id in transpose_parsed_output[0]
      if job_currently_exists:
        line_idx = transpose_parsed_output[0].index(job_id)
        old_line[1] = parsed_output[line_idx][1]  # Update status

      else:
        first_time_setting_to_inactive = (old_line[1] != "inactive")
        job_was_previously_pending = (old_line[1] == "PD")
        job_state = GetCommandOutput("seff %s | grep State:" % job_id, True)
        job_timed_out = "TIMEOUT" in job_state
        job_cancelled = "CANCELLED" in job_state
        intend_to_resubmit = old_line[2].split("/")[-1] in nonstop_sbatch_script

        # If a previously-pending job was canceled, then remove it from the history (we don't want this info in the file)
        if job_was_previously_pending and job_cancelled:
          old_line.clear()
          continue

        # Otherwise, set the job status to inactive, and resubmit a new one if desired.
        old_line[1] = "inactive"  # Update status
        if first_time_setting_to_inactive and job_timed_out and intend_to_resubmit:
          print("resubmitting %s" % old_line[2])
          RunCommand("sbatch " + old_line[2], True)

    merged_output = parsed_history_output

  else:
    merged_output = parsed_output

  # Clean up data -- remove leading white spaces
  merged_output = [[e.lstrip() for e in line] for line in merged_output]
  merged_output = [e for e in merged_output if len(e) > 0]  # get rid of empty line
  # merged_output = [[e.replace("Command=", "") for e in line] for line in merged_output]

  # Sort the list
  merged_output.sort()

  # Write jobs into file
  f = open(status_file_path, "w")
  f.write("JOBID, ST, COMMAND\n")
  f.write("\n".join([", ".join(line) for line in merged_output]))
  f.write("\n")
  f.close()

  # break
  if len(nonstop_sbatch_script) > 0:
    time.sleep(60)
  else:
    time.sleep(600)



