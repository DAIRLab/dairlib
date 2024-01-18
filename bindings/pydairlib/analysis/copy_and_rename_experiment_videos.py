import os
import pdb
import sys
import lcm
import glob
from datetime import datetime
from datetime import timedelta
import numpy as np

from lxml import objectify
from argparse import ArgumentParser

# add five hours to convert from EST to UTC
EASTERN_STANDARD_UTC_OFFSET = timedelta(hours=5)
epoch = datetime.utcfromtimestamp(0)


def read_creation_time(video_folder, video_id):
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


def get_logs(log_folder):
    logfiles = glob.glob(os.path.join(log_folder, 'lcmlog-[0-9]*'))
    log_data = {}
    for logname in logfiles:
        log = lcm.EventLog(logname, 'r')
        log.seek(0)
        timestamp_seconds = log.read_next_event().timestamp / 1e6
        log_data[timestamp_seconds] = logname

    return log_data


def find_path(matrix, starting_point, tol=1.0):
    n = len(matrix)
    m = len(matrix[0])
    i = starting_point[0]
    j = starting_point[1]

    path = []
    target_val = matrix[i][j]
    while i < (n) and j < (m):

        if np.abs(target_val - matrix[i][j]) < tol:
            path.append((i, j))
            i += 1
            j += 1
        elif matrix[i][j] < target_val:
            i += 1
        else:
            j += 1

    return path


def line_up(video_times, log_times):
    min_video_idx = 0
    max_video_idx = len(video_times) - 1

    # preprocessing to narrow down the list of potential videos to the same day
    # as the logs
    time_mismatch_tolerance = 7200
    while video_times[min_video_idx] + time_mismatch_tolerance < log_times[0]:
        min_video_idx += 1
    while video_times[max_video_idx] - time_mismatch_tolerance > log_times[-1]:
        max_video_idx -= 1
    valid_video_times = video_times[min_video_idx:(max_video_idx + 1)]

    # build the matrix
    m = len(log_times)
    n = len(valid_video_times)
    matrix = [
        [
            valid_video_times[i] - log_times[j] for j in range(m)
        ] for i in range(n)
    ]

    paths = [find_path(matrix, (i, 0)) for i in range(n)] + \
            [find_path(matrix, (j, 0)) for j in range(1, m)]

    path_ranking = sorted(paths, key=lambda a: len(a), reverse=True)
    best_path = path_ranking[0]
    idx_video, idx_log = zip(*best_path)

    video_times = [valid_video_times[i] for i in idx_video]
    log_times = [log_times[i] for i in idx_log]

    return video_times, log_times


def main():
    parser = ArgumentParser()
    parser.add_argument('--video_folder', type=str)
    parser.add_argument('--log_folder', type=str)
    parser.add_argument('--destination_folder', type=str)
    args = parser.parse_args()

    video_data = get_videos(args.video_folder)
    log_data = get_logs(args.log_folder)

    video_times = sorted(video_data)
    log_times = sorted(log_data)

    video_times, log_times = line_up(video_times, log_times)

    import pdb;
    pdb.set_trace()


if __name__ == '__main__':
    main()
