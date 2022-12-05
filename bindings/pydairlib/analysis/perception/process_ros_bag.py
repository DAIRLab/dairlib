try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import sys


def main():
    fname = sys.argv[1]

    topics = {}
    for topic, msg, t in rosbag.Bag(fname):
        if topic not in topics.keys():
            topics[topic] = 1
        else:
            topics[topic] += 1

    [print(f'{key}: {topics[key]} messages') for key in topics.keys()]


if __name__ == '__main__':
    main()