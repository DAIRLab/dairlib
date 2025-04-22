import lcm
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt


from pydairlib.analysis.process_lcm_log import get_log_data

from dairlib import lcmt_id_mpc_walking_debug, lcmt_robot_output

debug_channel = "ID_MPC_DEBUG"
debug_type = lcmt_id_mpc_walking_debug

plotting_palette = ["#011f5b", "#9f642d", "#af0000", "#b99aa0", "#666666", "#5583ab"]


def setup_plots():
    matplotlib.rcParams.update(matplotlib.rcParamsDefault)
    font = {'size': 20, 'family': 'serif'}
    matplotlib.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"
    matplotlib.rc('text.latex', preamble=r'\usepackage{underscore}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 1
    matplotlib.rcParams['axes.titlesize'] = 30
    matplotlib.rcParams['xtick.major.size'] = 15
    matplotlib.rcParams['xtick.major.width'] = 1
    matplotlib.rcParams['xtick.minor.size'] = 7
    matplotlib.rcParams['xtick.minor.width'] = 1
    plt.rcParams['axes.prop_cycle'] = plt.cycler(color=plotting_palette)


def process(debug_data):
    out = {
        't': [],
        'accepted': [],
        'constraint_viol': [],
        'cost': [],
        'step_size': [],
        'setup_time': [],
        'solve_time': [],
        'line_search_time': [],
        'total_step_time': [],
    }
    for msg in debug_data[debug_channel]:
        out['t'].append(msg.utime * 1e-6)
        out['accepted'].append(msg.ls_debug.accepted)
        out['constraint_viol'].append(msg.ls_debug.constraint_viol)
        out['cost'].append(msg.ls_debug.cost)
        out['step_size'].append(msg.ls_debug.step_size)
        out['setup_time'].append(msg.ls_debug.setup_time)
        out['solve_time'].append(msg.ls_debug.solve_time)
        out['line_search_time'].append(msg.ls_debug.line_search_time)
        out['total_step_time'].append(msg.ls_debug.total_step_time)

    return out


def dict_to_latex_table(data_dict, keys):

    # Calculate mean and std for each key
    means = {}
    stds = {}
    maxs = {}
    mins = {}

    for key in keys:
        values = np.array(data_dict[key])
        means[key] = np.mean(values)
        stds[key] = np.std(values)
        mins[key] = np.min(values)
        maxs[key] = np.max(values)

    def format_key(key):
        labels = key.split('_')[:-1]

        return " ".join(labels).title()

    # Generate LaTeX table
    latex_code = "\\begin{tabular}{|l|" + "c|" * len(keys) + "}\n"
    latex_code += "\\hline\n"

    # Header row
    latex_code += " & " + " & ".join([format_key(key) for key in keys]) + " \\\\\n"
    latex_code += "\\hline\n"

    # Mean row
    mean_values = [f"{means[key]:.3f}" for key in keys]
    latex_code += "Mean & " + " & ".join(mean_values) + " \\\\\n"

    # Std row
    std_values = [f"{stds[key]:.3f}" for key in keys]
    latex_code += "Std. & " + " & ".join(std_values) + " \\\\\n"

    # Mean row
    max_values = [f"{maxs[key]:.3f}" for key in keys]
    latex_code += "Max. & " + " & ".join(max_values) + " \\\\\n"

    # Mean row
    min_values = [f"{mins[key]:.3f}" for key in keys]
    latex_code += "Min. & " + " & ".join(min_values) + " \\\\\n"

    latex_code += "\\hline\n"
    latex_code += "\\end{tabular}"

    return latex_code


# Example usage
def print_latex_table(data_dict, keys):
    """
    Print the LaTeX table to the console
    """
    latex_table = dict_to_latex_table(data_dict, keys)
    print(latex_table)


def main():
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    data = get_log_data(log, {debug_channel: debug_type}, 0, -1, process)
    print_latex_table(
        data,
        ["setup_time", "solve_time", "line_search_time", "total_step_time"]
    )


if __name__ == '__main__':
    main()


