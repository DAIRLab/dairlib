try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import os
import sys


def split_log(fname, outfolder):
    topics = {}
    for topic, msg, t in rosbag.Bag(fname):
        if topic not in topics.keys():
            topic_fname = os.path.join(
                outfolder,
                f'{topic.replace("/", "_")[1:]}.bag'
            )
            topics[topic] = rosbag.Bag(topic_fname, 'w')
        topics[topic].write(topic, msg, t)

    for topic in topics.items():
        topic[1].close()


def main():
    fname = sys.argv[1]
    # outfolder = sys.argv[2]

    # split_log(fname, outfolder)

    topics = {}
    for topic, msg, t in rosbag.Bag(fname):
        if topic not in topics.keys():
            topics[topic] = 1
        else:
            topics[topic] += 1

    [print(f'{key}: {topics[key]} messages') for key in topics.keys()]


if __name__ == '__main__':
    main()