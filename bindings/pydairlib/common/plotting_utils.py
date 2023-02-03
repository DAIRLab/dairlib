import matplotlib.pyplot as plt
import plot_styler

"""
Plots data on the given plot_styler plot. 
:param data_dictionary: dictionary containing the data to be plotted. It should 
contain a time channel of length N and at least one data channel of size NxM
:param time_key: The dictionary key of the time channel in data_dictionary
:param time_slice: Slice object representing time indices to plot
:param keys_to_plot: List of keys (['a_key'] for a single key) to plot. 
data_dictionary[key] should be a Nx something array 
:param slices_to_plot: dictionary with the keys in keys_to_plot with what slice
each datapoint should be plotted for each data channel
:param legend_entries: Dictionary of list of strings r=which will become the
legend entry for each data channel - i.e. {'pos' : ['x', 'y', 'z']} if plotting 
one 3 dimensional data channel named 'pos'
"""


def make_plot(data_dictionary, time_key, time_slice, keys_to_plot,
              slices_to_plot, legend_entries, plot_labels, ps):
    legend = []
    for key in keys_to_plot:
        if key not in slices_to_plot:
            ps.plot(data_dictionary[time_key][time_slice],
                    data_dictionary[key][time_slice])
        else:
            ps.plot(data_dictionary[time_key][time_slice],
                    data_dictionary[key][time_slice, slices_to_plot[key]])
        if key in legend_entries:
            legend.extend(legend_entries[key])

    # plt.legend(legend)
    ps.add_legend(legend)
    plt.xlabel(plot_labels['xlabel'])
    plt.ylabel(plot_labels['ylabel'])
    plt.title(plot_labels['title'])


def make_plot_of_entire_series(data_dictionary, time_key, legend_entries,
                               plot_labels, ps):
    keys_to_plot = []
    for key in data_dictionary.keys():
        if key != time_key:
            keys_to_plot.append(key)

    make_plot(data_dictionary, time_key, slice(len(data_dictionary[time_key])),
              keys_to_plot, {}, legend_entries, plot_labels, ps)


def make_mixed_data_plot(data_dictionaries, time_keys, time_slices,
                         keys_to_plot, slices_to_plot,
                         legend_entries, plot_labels,
                         ps):
    legend = []
    for i, data_dictionary in enumerate(data_dictionaries):
        time_key = time_keys[i]
        time_slice = time_slices[i]
        for key in keys_to_plot[i]:
            if key not in slices_to_plot[i]:
                ps.plot(data_dictionary[time_key][time_slice],
                        data_dictionary[key][time_slice])
            else:
                ps.plot(data_dictionary[time_key][time_slice],
                        data_dictionary[key][time_slice, slices_to_plot[key]])
            legend.extend(legend_entries[key])

    ps.add_legend(legend)
    plt.xlabel(plot_labels['xlabel'])
    plt.ylabel(plot_labels['ylabel'])
    plt.title(plot_labels['title'])


def slice_to_string_list(slice_):
    if isinstance(slice_, slice):
        return [str(i) for i in range(slice_.start, slice_.stop,
                                      slice_.step if slice_.step else 1)]
    if isinstance(slice_, list):
        return [str(i) for i in slice_]
