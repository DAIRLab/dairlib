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

# It happened that I accidentally ran two identical jobs at the same time, so I wrote a function to check any duplication
def MakeSureThereIsNoDuplicatedJobs(current_output):
  output_with_only_running_jobs = [line for line in current_output if line[1] == "R"]
  output_with_only_running_jobs.sort()

  script_names = [line[2] for line in output_with_only_running_jobs]

  ids_of_cancelled_job = []
  names_of_cancelled_job = set()
  for name in script_names:
    if name not in names_of_cancelled_job:
      index_of_occurrences = [i for i in range(len(script_names)) if script_names[i] == name]
      for i in index_of_occurrences[1:]:
        print("cancelling a job with id=%s due to duplication" % output_with_only_running_jobs[i][0])
        RunCommand("scancel " + output_with_only_running_jobs[i][0], True)
        ids_of_cancelled_job.append(output_with_only_running_jobs[i][0])
        names_of_cancelled_job.add(name)
  return ids_of_cancelled_job

def ParseJobTable(txt):
  parsed = txt.split("\n")  # split by newline
  parsed = [e for e in parsed[1:] if len(e) > 0]  # ignore the first line and get rid of empty line
  parsed = [e.split(", ") for e in parsed]  # split again by ", "
  return parsed


# Parameters
status_file_path = "../job_status.txt"
nonstop_sbatch_script = []  # re-submit scripts if added. Only file name is required (don't need directory)

# Some setups
nonstop_sbatch_script = [e.split("/")[-1] for e in nonstop_sbatch_script]  # Get rid of directory. Only keep file name

while True:
  # Read current status
  '''
  Example output of `squeue -u yminchen -o "%i, %t, %o"`
  JOBID, ST, COMMAND
  150102, PD, /mnt/beegfs/scratch/yminchen/dairlib/examples/goldilocks_models/cluster_scripts/20220202_rom17_much_smaller_range__only_walking_forward__more_forward__lower_height.bash
  150038, R, /mnt/beegfs/scratch/yminchen/dairlib/examples/goldilocks_models/cluster_scripts/model_optimization_on_cluster2_20220201_rom16_much_smaller_range__only_walking_forward__more_forward.bash
  150090, R, /mnt/beegfs/scratch/yminchen/dairlib/examples/goldilocks_models/cluster_scripts/model_optimization_on_cluster2_20220202_rom17_much_much_smaller_range__only_walking_forward__more_forward.bash
  '''
  current_output = ParseJobTable(GetCommandOutput("squeue -u yminchen -o \"%i, %t, %o\"", True))

  # Remove plotting or debugging jobs from the list
  current_output = [e for e in current_output if "/mnt/beegfs/scratch/yminchen" in e[2]]

  # Cancel duplicated jobs
  ids_of_cancelled_job = MakeSureThereIsNoDuplicatedJobs(current_output)
  current_output = [e for e in current_output if e[0] not in ids_of_cancelled_job]

  merged_output = []
  if os.path.exists(status_file_path):
    # Merge the current status with the old status
    f = open(status_file_path, "r")
    history_output = ParseJobTable(f.read())
    f.close()

    # Add new job to history
    n_new = 0
    for line in current_output:
      job_exist_in_history = False if (len(list(zip(*history_output))) == 0) else (line[0] in list(zip(*history_output))[0])  # check if the new job id exists in the history
      if not job_exist_in_history:
        history_output.append(line)
        n_new = n_new + 1 

    # Update old job status
    for old_line in history_output[:-n_new]:
      job_id = old_line[0]

      current_output_T = [list(x) for x in zip(*current_output)]
      job_currently_exists = False if (len(current_output) == 0) else (job_id in current_output_T[0])
      if job_currently_exists:
        line_idx = current_output_T[0].index(job_id)
        old_line[1] = current_output_T[1][line_idx]  # Update status

      else:
        first_time_setting_to_inactive = (old_line[1] != "inactive")
        job_was_previously_pending = (old_line[1] == "PD")
        job_state = GetCommandOutput("seff %s | grep State:" % job_id, True)
        job_timed_out = "TIMEOUT" in job_state
        job_cancelled = "CANCELLED" in job_state
        intend_to_resubmit = old_line[2].split("/")[-1] in nonstop_sbatch_script

        # If a previously-pending job was canceled, then remove it from the history (we don't want this info in the history)
        if job_was_previously_pending and job_cancelled:
          old_line.clear()
          continue

        # Otherwise, set the job status to inactive, and resubmit a new one if desired.
        old_line[1] = "inactive"  # Update status
        if first_time_setting_to_inactive and job_timed_out and intend_to_resubmit:
          print("resubmitting %s" % old_line[2])
          RunCommand("sbatch " + old_line[2], True)

    merged_output = history_output

  else:
    merged_output = current_output

  # Clean up data -- remove leading white spaces
  merged_output = [[e.lstrip() for e in line] for line in merged_output]
  merged_output = [e for e in merged_output if len(e) > 0]  # get rid of empty line

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



