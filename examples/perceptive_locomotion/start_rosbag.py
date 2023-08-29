import subprocess
import os
import glob
import codecs
from datetime import date

import io
import sys
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


def write_git_diff(repo_dir,  log_dir, log_num):
    git_diff = subprocess.check_output(['git', 'diff'], cwd=repo_dir)
    commit_tag = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repo_dir
    )

    repo_name = repo_dir.split('/')[-1]
    commit_path = os.path.join(log_dir, f'{repo_name}_commit_tag-{log_num}')
    with open(commit_path, 'w') as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])


def rosbagger_main(topics_filename):
    topic_data = load(io.open(topics_filename, 'r'), Loader=Loader)
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{os.getenv('HOME')}/logs/{year}/{curr_date}"

    user = os.getenv('HOME').split('/')[-1]
    workspace_subdir = "" if user == 'brian' else "/brian"

    elevation = f"{os.getenv('HOME')}/workspace{workspace_subdir}/" \
                f"catkin_workspace/src/elevation_mapping_cupy"
    cassie_ros = f"{os.getenv('HOME')}/workspace{workspace_subdir}/" \
                 f"catkin_workspace/src/cassie_ros"
    if not os.path.isdir(logdir):
        os.makedirs(logdir)

    os.chdir(logdir)

    # Line this up with the mpc logs
    current_logs = sorted(glob.glob('lcmlog-mpc-*'))
    if current_logs:
        last_log = int(current_logs[-1].split('-')[-1])
        log_num = f'{last_log:02}'
    else:
        log_num = '00'

    write_git_diff(elevation, logdir, log_num)
    write_git_diff(cassie_ros, logdir, log_num)

    rosbag_cmd = ['rosbag', 'record', '-o',
                  logdir + f'/cassie-rosbag-{log_num}'] + topic_data['topics']
    subprocess.run(rosbag_cmd)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        rosbagger_main(sys.argv[1])
    else:
        rosbagger_main("examples/perceptive_locomotion/logtopics.yaml")
