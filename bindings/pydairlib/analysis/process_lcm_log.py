def get_log_data(lcm_log, lcm_channels, end_time, data_processing_callback, *args,
                 **kwargs):
    """
    Parses an LCM log and returns data as specified by a callback function
    :param lcm_log: an lcm.EventLog object
    :param lcm_channels: dictionary with entries {channel : lcmtype} of channels
    to be read from the log
    :param data_processing_callback: function pointer which takes as arguments
     (data, args, kwargs) where data is a dictionary with
     entries {CHANNEL : [ msg for lcm msg in log with msg.channel == CHANNEL ] }
    :param args: positional arguments for data_processing_callback
    :param kwargs: keyword arguments for data_processing_callback
    :return: return args of data_processing_callback
    """

    data_to_process = {}
    print('Processing LCM log (this may take a while)...')
    t = lcm_log.read_next_event().timestamp
    lcm_log.seek(0)
    for event in lcm_log:
        if event.channel in lcm_channels:
            if event.channel in data_to_process:
                data_to_process[event.channel].append(
                    lcm_channels[event.channel].decode(event.data))
            else:
                data_to_process[event.channel] = \
                    [lcm_channels[event.channel].decode(event.data)]

        if event.eventnum % 50000 == 0:
            print(f'processed {(event.timestamp - t)*1e-6:.1f}'
                  f' seconds of log data')

        if 0 < end_time <= (event.timestamp - t)*1e-6:
            break
    return data_processing_callback(data_to_process, *args, *kwargs)


def get_log_summary(lcm_log):
    channel_names_and_msg_counts = {}
    for event in lcm_log:
        if event.channel not in channel_names_and_msg_counts:
            channel_names_and_msg_counts[event.channel] = 1
        else:
            channel_names_and_msg_counts[event.channel] = \
            channel_names_and_msg_counts[event.channel] + 1
    return channel_names_and_msg_counts


def print_log_summary(filename, log):
    print(f"Channels in {filename}:\n")
    summary = get_log_summary(log)
    for channel, count in summary.items():
        print(f"{channel}: {count:06} messages")


def passthrough_callback(data, *args, **kwargs):
    return data

import dairlib
import dairlib.lcmt_robot_output


channels = {
    "FRANKA_OUTPUT": dairlib.lcmt_robot_output,
}

def processing_callback(data, channel):
    x = []
    y = []
    for msg in data[channel]:
        x.append(msg.position[11])
        y.append(msg.position[12])
    return x, y

def main():
    import lcm
    import sys
    import matplotlib.pyplot as plt
    import numpy as np
    import scipy.io

    logfile = "/home/alpaydinoglu/workspace/dairlib/example_log"
    log = lcm.EventLog(logfile, "r")
    # print_log_summary(logfile, log)
    x, y = get_log_data(log, channels, -1, processing_callback, "FRANKA_OUTPUT")
    plt.plot(x, y)

    circle2 = plt.Circle((0.55, 0), 0.1, color='b', fill=False)
    plt.gca().add_patch(circle2)

    plt.show()


    print("creating mat file")
    mdic = {"x": x, "y": y}
    scipy.io.savemat('xy_ball.mat', mdic)
    print("finished creating mat file")


if __name__ == "__main__":
    main()
