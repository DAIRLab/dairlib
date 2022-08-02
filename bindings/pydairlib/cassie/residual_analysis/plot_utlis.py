import wandb
import matplotlib.pylab as plt
import numpy as np
from matplotlib import patches

class PlotViewlizer():

    def __init__(self, project_name):
        wandb.init(project=project_name)
    def add_info(self, log):
        wandb.log(log)

def get_data_of_selected_time(original_data, start_time , end_time):
    cut_data = []
    for datum in original_data:
        if not (datum['t'] >= start_time and datum['t'] <= end_time):
                continue
        cut_data.append(datum)
    return cut_data

def plot_spring_force_vs_time(processed_data, start_time, end_time):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    x = [x["t"] for x in cut_data]
    legends = ["knee_joint_left", "knee_joint_right", "ankle_spring_joint_left", "ankle_spring_joint_right"]
    ys = []
    for l in legends:
        ys.append([x["spring_force_of_best_spring_model"][l] for x in cut_data])
    x = np.array(x)
    ys = np.array(ys)

    plot_given_keys(cut_data, x, ys, legends, "t(s)", "force (N*m)", "Fitted best spring force")

def plot_contact_background(cut_data, min_ys, max_ys):
    left_on_ground = []
    right_on_ground = []
    left_start = None
    right_start = None

    for i in range(len(cut_data)):
        datum = cut_data[i]
        if left_start is None:
            if datum["is_contact"]["left"] == 1:
                left_start = datum["t"]
        else:
            if datum["is_contact"]["left"] == 0 or i == len(cut_data) - 1:
                left_on_ground.append((left_start, datum["t"]))
                left_start = None

        if right_start is None:
            if datum["is_contact"]["right"] == 1:
                right_start = datum["t"]
        else:
            if datum["is_contact"]["right"] == 0 or i == len(cut_data) - 1:
                right_on_ground.append((right_start, datum["t"]))
                right_start = None

    for (start_time, end_time) in left_on_ground:
        rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="b", alpha=0.2)
        plt.gca().add_patch(rect)
    
    for (start_time, end_time) in right_on_ground:
        rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="y", alpha=0.2)
        plt.gca().add_patch(rect)

def plot_given_keys(cut_data, x, ys, legends, xlabel, ylabel, title):

    plt.figure()

    for i in range(len(ys)):
        plt.plot(x, ys[i], label=legends[i])
        plt.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    max_ys = np.max(ys)
    min_ys = np.min(ys)
    plot_contact_background(cut_data, min_ys, max_ys)
    
    plt.show()