

def get_log_data(lcm_log, lcm_channels, data_processing_callback, *args,
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
    for event in lcm_log:
        if event.channel in lcm_channels:
            if event.channel in data_to_process:
                data_to_process[event.channel].append(
                    lcm_channels[event.channel].decode(event.data))
            else:
                data_to_process[event.channel] = \
                    [lcm_channels[event.channel].decode(event.data)]

    return data_processing_callback(data_to_process, *args, *kwargs)


def passthrough_callback(data, *args, **kwargs):
    return data
