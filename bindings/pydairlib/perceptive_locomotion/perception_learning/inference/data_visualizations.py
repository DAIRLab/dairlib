import numpy as np
from matplotlib import pyplot as plt


def load_training_data():
    data = np.load(
        'bindings/pydairlib/perceptive_locomotion/perception_learning/tmp/'
        'data.npz',
        allow_pickle=True
    )
    data = data['arr_0']
    return data


def hist_residuals(data):
    residuals = [datapoint['residual'] for datapoint in data]
    plt.hist(residuals, bins=20)


def box_residuals(data):
    residuals = [datapoint['residual'] for datapoint in data]
    plt.boxplot(residuals)
    plt.ylabel('Residual Value')
    plt.title('Distribution of Residual Data (Box Plot)')


def box_alip_state_norm(data):
    norms = [np.linalg.norm(datapoint['x_k']) for datapoint in data]
    plt.boxplot(norms)
    plt.title('Distribution of Norm of x_k')
    plt.ylabel('|x_k|')


def box_next_alip_state_norm(data):
    norms = [np.linalg.norm(datapoint['x_kp1']) for datapoint in data]
    plt.boxplot(norms)
    plt.title('Distribution of Norm of x_kp1')
    plt.ylabel('|x_k|')


def box_v_k(data):
    vk = [datapoint['V_k'] for datapoint in data]
    plt.boxplot(vk)
    plt.title('Distribution of V_k')
    plt.ylabel('V_k')


def box_v_kp1(data):
    vkp1 = [datapoint['V_kp1'] for datapoint in data]
    plt.boxplot(vkp1)
    plt.title('Distribution of V_kp1')
    plt.ylabel('V_kp1')


def make_fig(data, plotter):
    plt.figure()
    plotter(data)


if __name__ == '__main__':
    training_data = load_training_data()
    for plotter in [box_residuals,  box_alip_state_norm, box_v_k]:
        make_fig(training_data, plotter)
    plt.show()