import matplotlib.pyplot as plt
import plot_styler


def make_plot(data_dictionary, time_key, time_slice, keys_to_plot,
              slices_to_plot, legend_entries, plot_labels,
              ps):
    legend = []
    for key in keys_to_plot:
        if key not in slices_to_plot:
            ps.plot(data_dictionary[time_key][time_slice],
                    data_dictionary[key][time_slice])
        else:
            ps.plot(data_dictionary[time_key][time_slice],
                    data_dictionary[key][time_slice, slices_to_plot[key]])
        legend.extend(legend_entries[key])

    plt.legend(legend)
    plt.xlabel(plot_labels['xlabel'])
    plt.ylabel(plot_labels['ylabel'])
    plt.title(plot_labels['title'])
    ps.set_default_styling()


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
                        data_dictionary[key][time_slice,
                                             slices_to_plot[i][key]])
            legend.extend(legend_entries[i])

    ps.add_legend(legend)
    plt.xlabel(plot_labels['xlabel'])
    plt.ylabel(plot_labels['ylabel'])
    plt.title(plot_labels['title'])
    ps.set_default_styling()
