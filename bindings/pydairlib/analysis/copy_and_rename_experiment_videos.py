import os
import pdb
import sys
import lcm
import glob
from datetime import datetime
from datetime import timedelta

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
    while i < n and j < m:
        if abs(target_val - matrix[i][j]) < tol:
            path.append((i, j))
            i += 1
            j += 1
        elif matrix[i][j] < target_val:
            i += 1
        else:
            j += 1

    return path


def line_up_timestamps(times_a, times_b, tol=1.0):
    """
    Finds the corresponding elements of two lists of timestamps, where there
    might be a constant offset between the two time sources, and each list may
    have any number of extra or missing elements, at any position.


    :param tol: tolerance for differences in timestamps (seconds)
    :param times_a: first sequence of timestamps
    :param times_b: second sequence of timestamps
    :return: (times_a_elements, times_b_elements), the common subsequence of
    timestamps, as elements of each original list
    """

    times_a = sorted(times_a)
    times_b = sorted(times_b)

    # Build a matrix of the difference between the two time stamp lists
    n = len(times_a)
    m = len(times_b)
    matrix = [
        [
            times_a[i] - times_b[j] for j in range(m)
        ] for i in range(n)
    ]

    # Find the constant offset with the largest number of valid correspondences.
    # Count smartly by making a path through the matrix
    paths = [find_path(matrix, (i, 0), tol=tol) for i in range(n)] + \
            [find_path(matrix, (0, j), tol=tol) for j in range(1, m)]

    path_ranking = sorted(paths, key=lambda a: len(a), reverse=True)
    best_path = path_ranking[0]
    idx_a, idx_b = zip(*best_path)

    times_a_elements = [times_a[i] for i in idx_a]
    times_b_elements = [times_b[i] for i in idx_b]

    return times_a_elements, times_b_elements


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

    # preprocessing to narrow down the list of potential videos to the same day
    # as the logs
    time_mismatch_tolerance = 7200
    min_video_idx = 0
    max_video_idx = len(log_times)

    while video_times[min_video_idx] + time_mismatch_tolerance < log_times[0]:
        min_video_idx += 1
    while video_times[max_video_idx] - time_mismatch_tolerance > log_times[-1]:
        max_video_idx -= 1
    valid_video_times = video_times[min_video_idx:(max_video_idx + 1)]

    video_times, log_times = line_up_timestamps(valid_video_times, log_times)

    import pdb;
    pdb.set_trace()


if __name__ == '__main__':
    main()
