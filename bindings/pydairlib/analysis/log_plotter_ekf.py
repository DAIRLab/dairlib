import sys
import lcm
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import code
import numpy as np

import dairlib
from process_lcm_log import get_log_data
from mbp_plotting_utils import process_ekf_channel
from pydairlib.common import plot_styler, plotting_utils

ekf_channel = {'EKF_DEBUG_OUT': dairlib.lcmt_ekf_debug_out}


def process_ekf(data):
    P_prop = []
    P_corr = []
    X_prop = []
    X_corr = []
    Theta_prop = []
    Theta_corr = []
    for msg in data['EKF_DEBUG_OUT']:
        P_prop.append(msg.P_prop)
        P_corr.append(msg.P_corr)
        X_prop.append(msg.X_prop)
        X_corr.append(msg.X_corr)
        Theta_prop.append(msg.Theta_prop)
        Theta_corr.append(msg.Theta_corr)

    return {'P_prop': np.array(P_prop),
            'P_corr': np.array(P_corr),
            'X_prop': np.array(X_prop),
            'X_corr': np.array(X_corr),
            'Theta_prop': np.array(Theta_prop),
            'Theta_corr': np.array(Theta_corr)}


def animate_covariance(P):
    fig = plt.figure(figsize=(15,15))
    ax = plt.axes()

    def frame(i):
        ax.clear()
        plot = ax.imshow(P[100*i], cmap='hot', interpolation='nearest')
        return plot

    anim = animation.FuncAnimation(fig, frame, frames=P.shape[0], repeat=False)
    return anim


def main():
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    ekf_data = get_log_data(log, ekf_channel, -1, process_ekf_channel, "EKF_DEBUG_OUT")
    plt.plot(ekf_data['t'], ekf_data['v_prop'])

    # anim = animate_covariance(ekf_data['P_prop'])
    plt.show()


if __name__ == '__main__':
    main()
