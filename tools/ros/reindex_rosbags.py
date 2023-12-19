import os
import sys
import glob
import subprocess


def main(target_dir):
    """
        Script to reindex all rosbags in a directory
        Usage: python3 tools/workspace/ros/reindex_rosbags.py /path/to/data/
    """
    assert(os.path.isdir(target_dir))

    recursive = input("recursive? (y/n): ").strip().lower() == "y"
    bag_list = glob.glob(f'{target_dir}/**/*.bag', recursive=recursive) + \
                glob.glob(f'{target_dir}/**/*.bag.active', recursive=recursive)

    for bag in bag_list:
        subprocess.run(['rosbag', 'reindex', bag])


if __name__ == "__main__":
    try:
        target = sys.argv[1]
    except IndexError:
        target = input("Please enter a target directory: ")

    main(target)
