import os
import pdb
import sys
import lcm
import glob
from shutil import copy2
from datetime import datetime
from datetime import timedelta

from lxml import objectify
from argparse import ArgumentParser

# add five hours to convert from EST to UTC
EASTERN_STANDARD_UTC_OFFSET = timedelta(hours=5)
epoch = datetime.utcfromtimestamp(0)

from pydairlib.analysis.file_utils import line_up_timestamps


def read_creation_time(video_folder, video_id):
    if not os.path.exists(video_folder):
        raise RuntimeError(f'video folder {video_folder} does not exist.')
    with open(os.path.join(video_folder, f'{video_id}M01.XML'), 'rb') as f:
        contents = f.read()
        tree = objectify.fromstring(contents)
        datestring = tree.CreationDate.get('value')

    date_format = '%Y-%m-%dT%H:%M:%S-05:00'
    creation_time = (datetime.strptime(datestring, date_format) +
                     EASTERN_STANDARD_UTC_OFFSET)

    return (creation_time - epoch).total_seconds()


def get_videos(video_folder):
    videos = glob.glob(os.path.join(video_folder, '*.MP4'))
    ids = [v.split('/')[-1].split('.')[0] for v in videos]

    video_data = {
        read_creation_time(video_folder, i): v for (i, v) in zip(ids, videos)
    }
    return video_data


def get_logs(log_folder, pattern):
    logfiles = glob.glob(os.path.join(log_folder, pattern))
    log_data = {}
    for logname in logfiles:
        log = lcm.EventLog(logname, 'r')
        log.seek(0)
        timestamp_seconds = log.read_next_event().timestamp / 1e6
        log_data[timestamp_seconds] = logname

    return log_data


def get_cassie_logs(log_folder):
    return get_logs(log_folder, 'lcmlog-[0-9]*')


def get_laptop_logs(log_folder):
    return get_logs(log_folder, 'lcmlog-laptop-[0-9]*')


def get_destination_sub_folder_path(destination_folder, log_folder):
    chunks = log_folder.split('/')
    date = chunks[-1]
    year = chunks[-2]

    if year not in destination_folder:
        destination_folder = os.path.join(destination_folder, year)

    if date not in destination_folder:
        destination_folder = os.path.join(destination_folder, date)
    return destination_folder


def main():
    parser = ArgumentParser()
    parser.add_argument('--video_folder', type=str)
    parser.add_argument('--log_folder', type=str)
    parser.add_argument(
        '--destination_folder', type=str,
        default='/media/brian/tb2/cassie_backup/logs/unified'
    )
    args = parser.parse_args()

    video_data = get_videos(args.video_folder)
    log_data = get_cassie_logs(args.log_folder)
    laptop_data = get_laptop_logs(args.log_folder)

    video_times = sorted(video_data)
    laptop_times = sorted(laptop_data)
    log_times = sorted(log_data)

    video_times, log_times_video = line_up_timestamps(video_times, log_times)
    laptop_times, log_times_laptop = line_up_timestamps(laptop_times, log_times)

    destination = get_destination_sub_folder_path(
        args.destination_folder,
        args.log_folder
    )

    for t in log_times:
        logname = log_data[t].split('/')[-1]
        new_folder_name = os.path.join(destination, logname)
        os.makedirs(new_folder_name, exist_ok=True)

        copy2(log_data[t], new_folder_name)

        if t in log_times_video:
            idx = log_times_video.index(t)
            video = video_data[video_times[idx]]
            dest = os.path.join(new_folder_name, logname + '.mp4')
            print(f'Copying {video} to {dest}')
            copy2(video, dest)

        if t in log_times_laptop:
            idx = log_times_laptop.index(t)
            laptop_log = laptop_data[laptop_times[idx]]
            dest = new_folder_name
            print(f'Copying {laptop_log} to {dest}')
            copy2(laptop_log, dest)


if __name__ == '__main__':
    main()
