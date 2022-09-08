import wandb
import matplotlib.pylab as plt
import numpy as np
from matplotlib import patches

class WandbProcessor():

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

def plot_spring_force_vs_time(processed_data, start_time, end_time, directory=None, log_name=None, is_show=False):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    x = [x["t"] for x in cut_data]
    legends = ["knee_joint_left", "knee_joint_right", "ankle_spring_joint_left", "ankle_spring_joint_right"]
    ys = []
    for l in legends:
        ys.append([x["spring_force_of_best_spring_model"][l] for x in cut_data])
    x = np.array(x)
    ys = np.array(ys)

    plot_given_keys_vs_time(cut_data, x, ys, legends, "t(s)", "force (N*m)", "spring_force_vs_t", directory=directory, log_name=log_name, is_show=is_show)

def plot_spring_force_vs_q(processed_data, start_time, end_time, is_sorted=False, directory=None, log_name=None, is_show=False):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    legends = ["knee_joint_left", "knee_joint_right", "ankle_spring_joint_left", "ankle_spring_joint_right"]
    plt.figure()
    for l in legends:
        x = [x["q"][l] for x in cut_data]
        y = [x["spring_force_of_best_spring_model"][l] for x in cut_data]
        if is_sorted:
            x_y = []
            for i in range(len(x)):
                x_y.append((x[i],y[i]))
            x_y.sort(key=lambda x: x[0])
            x = [x[0] for x in x_y]
            y= [x[1] for x in x_y]
        plt.plot(x,y,label=l)
    plt.xlabel("q(rad)")
    plt.ylabel("spring force(N*m)")
    plt.title("Spring force vs spring deflection")
    plt.legend()
    if not directory or not log_name:
        if is_show:
            plt.show()
    else:
        plt.savefig(directory+"/spring_force_vs_q_" + log_name + ".png")

def plot_joint_residuals_vs_time(processed_data, start_time, end_time, joint_name, directory=None, log_name=None, is_show=False):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    x = [x["t"] for x in cut_data]
    ys = []
    ys.append([x["v_dot_gt"][joint_name] for x in cut_data])
    ys.append([x["v_dot_best_spring_model"][joint_name] for x in cut_data])
    x = np.array(x)
    ys = np.array(ys)
    legends = ["ground truth", "estimated"]
    plot_given_keys_vs_time(cut_data, x, ys, legends, "t(s)", "acc(rad/s^2)", "vdot_of_{}".format(joint_name), directory=directory, log_name=log_name, is_show=is_show)

def plot_joint_effort_vs_time(processed_data, start_time, end_time, joint_names, directory=None, log_name=None, is_show=False):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    x = [x["t"] for x in cut_data]
    ys = []
    for joint_name in joint_names:
        ys.append([x["u"][joint_name] for x in cut_data])
    x = np.array(x)
    ys = np.array(ys)
    plot_given_keys_vs_time(cut_data, x, ys, joint_names, "t(s)", "u(N*m)", "joint_efforts_vs_t", directory=directory, log_name=log_name, colors=['r', 'b', 'g', 'y', 'k', 'brown', 'pink', 'grey'], is_show=is_show)

def construct_plot_datum_pair_for_gt_and_est_vs_time(t, gt, est, title, ylabel):
    """
    The function will construct organized data struct for plot,
    """

    datum = {"x":t,
            "ys":[{"value":gt, "legend":"ground truth", "color":"green"},
                    {"value":est, "legend":"estimated", "color":"red"}],
            "title":title,
            "ylabel":ylabel,
            "xlabel":"t"
            }
    return datum

def plot_by_list_of_dictionaries(data, is_show=False):
    """
    The function will create figures as same length of the lenth of data,

    The data is a list, each element in it is dictionary for construct a single figure
    """
    for datum in data:
        plt.figure()
        x = datum["x"]
        ys = datum["ys"]
        for y in ys:
            if "color" in y:
                plt.plot(x, y["value"], label=y["legend"], color=y["color"])
            else:
                plt.plot(x, y["value"], label=y["legend"],)
        plt.legend()
        if "ylabel" in datum:
            plt.ylabel(datum["ylabel"])
        if "xlabel" in datum:
            plt.xlabel(datum["xlabel"])
        if "title" in datum:
            plt.title(datum["title"])         
    if is_show:
        plt.show()

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

    is_draw_label_left = False
    is_draw_label_right = False
    for (start_time, end_time) in left_on_ground:
        if not is_draw_label_left:
            rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="pink", alpha=0.2, label="left foot on ground")
            is_draw_label_left = True
        else:
            rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="pink", alpha=0.2)
        plt.gca().add_patch(rect)
    
    for (start_time, end_time) in right_on_ground:
        if not is_draw_label_right:
            rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="y", alpha=0.2, label="right foot on ground")
            is_draw_label_right = True
        else:
            rect = patches.Rectangle((start_time, min_ys), end_time - start_time, max_ys-min_ys, color="y", alpha=0.2)
        plt.gca().add_patch(rect)

def plot_joint_velocity_vs_time(processed_data, start_time, end_time, joint_name, is_show=False):
    cut_data = get_data_of_selected_time(processed_data, start_time, end_time)
    x = [x["t"] for x in cut_data]
    ys = []
    ys.append([x["v"][joint_name] for x in cut_data])
    x = np.array(x)
    ys = np.array(ys)
    legends = ["v"]
    plot_given_keys_vs_time(cut_data, x, ys, legends, "t(s)", "v(rad/s)", "velocity_of_{}".format(joint_name), is_show=is_show)

def plot_given_keys_vs_time(cut_data, x, ys, legends, xlabel, ylabel, title, colors=None, directory=None, log_name=None, is_show=False):

    plt.figure()

    max_ys = np.max(ys)
    min_ys = np.min(ys)
    plot_contact_background(cut_data, min_ys, max_ys)

    for i in range(len(ys)):
        if colors:
            plt.plot(x, ys[i], label=legends[i], color=colors[i % len(colors)])
        else:
            plt.plot(x, ys[i], label=legends[i])
        plt.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    max_ys = np.max(ys)
    min_ys = np.min(ys)
    plot_contact_background(cut_data, min_ys, max_ys)

    if not directory or not log_name:
        if is_show:
            plt.show()
    else:
        plt.savefig(directory+"/" + title +"_" + log_name + ".png")
