import numpy as np
from math import nan

# Class to easily convert list of lcmt_osc_tracking_data_t to numpy arrays
class lcmt_osc_tracking_data_t:
    def __init__(self, gap_threshold=0.01):
        self.t_thresh=gap_threshold
        self.t = []
        self.y_dim = 0
        self.name = ""
        self.is_active = []
        self.y = []
        self.y_des = []
        self.error_y = []
        self.ydot = []
        self.ydot_des = []
        self.error_ydot = []
        self.yddot_des = []
        self.yddot_command = []
        self.yddot_command_sol = []

    def append(self, msg, t):
        self.y_dim = len(msg.y)
        self.ydot_dim = len(msg.ydot)

        # If there is a large gap between tracking datas, append 
        # NaNs as a mask for plotting to avoid fictitious lines
        # appearing in plots
        if self.t and (t - self.t[-1]) > self.t_thresh:
            self.t.append(nan)
            self.is_active.append(nan)
            self.y.append([nan for _ in range(self.y_dim)])
            self.y_des.append([nan for _ in range(self.y_dim)])
            self.error_y.append([nan for _ in range(self.ydot_dim)])
            self.ydot.append([nan for _ in range(self.ydot_dim)])
            self.ydot_des.append([nan for _ in range(self.ydot_dim)])
            self.error_ydot.append([nan for _ in range(self.ydot_dim)])
            self.yddot_des.append([nan for _ in range(self.ydot_dim)])
            self.yddot_command.append([nan for _ in range(self.ydot_dim)])
            self.yddot_command_sol.append([nan for _ in range(self.ydot_dim)])

        self.t.append(t)
        self.is_active.append(msg.is_active)
        self.y.append(msg.y)
        self.y_des.append(msg.y_des)
        self.error_y.append(msg.error_y)
        self.ydot.append(msg.ydot)
        self.ydot_des.append(msg.ydot_des)
        self.error_ydot.append(msg.error_ydot)
        self.yddot_des.append(msg.yddot_des)
        self.yddot_command.append(msg.yddot_command)
        self.yddot_command_sol.append(msg.yddot_command_sol)

    def convertToNP(self):
        self.t = np.array(self.t)
        self.is_active = np.array(self.is_active)
        self.y = np.array(self.y)
        self.y_des = np.array(self.y_des)
        self.error_y = np.array(self.error_y)
        self.ydot = np.array(self.ydot)
        self.ydot_des = np.array(self.ydot_des)
        self.error_ydot = np.array(self.error_ydot)
        self.yddot_des = np.array(self.yddot_des)
        self.yddot_command = np.array(self.yddot_command)
        self.yddot_command_sol = np.array(self.yddot_command_sol)

# Helper class to easily get a list of acceleration tracking costs, 
# Setting the cost to zero when a tracking data is not active
class osc_tracking_cost():

    def __init__(self, tracking_data_names):

        self.tracking_costs = {}
        for name in tracking_data_names:
            self.tracking_costs[name] = []

    def append(self, tracking_data_list, tracking_cost_list):
        for name, cost in zip(tracking_data_list, tracking_cost_list):
            self.tracking_costs[name].append(cost)

        for name in self.tracking_costs:
            if name not in tracking_data_list:
                self.tracking_costs[name].append(0.0)

    def convertToNP(self):
        for name in self.tracking_costs:
            self.tracking_costs[name] = np.array(self.tracking_costs[name])
        return self.tracking_costs


# Helper class to easily get a list of acceleration tracking costs,
# Setting the cost to zero when a tracking data is not active
class osc_regularlization_tracking_cost():

    def __init__(self, regularization_data_names):

        self.regularization_costs = {}
        for name in regularization_data_names:
            self.regularization_costs[name] = []

    def append(self, regularization_cost_names_list, regularization_cost_list):
        for name, cost in zip(regularization_cost_names_list, regularization_cost_list):
            self.regularization_costs[name].append(cost)

    def convertToNP(self):
        for name in self.regularization_costs:
            self.regularization_costs[name] = np.array(self.regularization_costs[name])
        return self.regularization_costs
