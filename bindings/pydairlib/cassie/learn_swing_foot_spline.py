import pickle
import time
import numpy as np
from json import load, dump
import nevergrad as ng
from matplotlib import pyplot as plt

from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.swing_foot_env import make_swing_ft_env, \
    get_default_params
from pydairlib.common.plot_styler import PlotStyler
from pydrake.trajectories import PiecewisePolynomial


class SwingFootSplineOptimizer:
    def __init__(self, budget):
        self.budget = budget
        self.env = make_swing_ft_env()
        self.default_action = get_default_params()
        self.date_prefix = time.strftime("%Y_%m_%d_%H_&m")
        self.data_folder = 'bindings/pydairlib/cassie/data/'

    def get_single_reward(self, action):
        self.env.reset()
        while self.env.current_time < 2.4:
            _, _, t, _ = self.env.step(action)
            if t:
                self.env = make_swing_ft_env()
                break
        return -self.env.cumulative_reward

    def save_params(self, folder, params, budget):
        with open(folder + self.date_prefix + '_' + str(budget) + '.pkl', 'wb') as f:
            pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

    def load_params(self, param_file, folder):
        with open(folder + param_file + '.pkl', 'rb') as f:
            return pickle.load(f)

    def learn_spline(self):
        self.param_space = \
            ng.p.Array(lower=-0.2, upper=0.2, shape=(len(self.default_action),))
        optimizer = ng.optimizers.NGOpt(parametrization=self.param_space,
                                           budget=self.budget)
        optimizer.register_callback("tell", ng.callbacks.ProgressBar())
        params = optimizer.minimize(self.get_single_reward)
        self.save_params(self.data_folder, params, self.budget)


def params_to_traj(params, duration):
    n_knot = int((params.shape[0] - 6) / 3)
    T = np.linspace(0, duration, n_knot)
    knots = np.zeros((3, n_knot))
    for i in range(n_knot):
        knots[:, i] = params[3*i:3*i+3]
    pp = PiecewisePolynomial. CubicWithContinuousSecondDerivatives(
        T, knots, params[-6:-3], params[-3:])
    return pp


def params_to_xyz_samples(params, duration, n):
    pp = params_to_traj(params, duration)
    t = np.linspace(0, duration, n)
    x = np.zeros(t.shape)
    z = np.zeros(t.shape)
    y = np.zeros(t.shape)
    for i in range(t.shape[0]):
        xyz = pp.value(t[i])
        x[i] = xyz[0]
        y[i] = xyz[1]
        z[i] = xyz[-1]
    return x, y, z


def plot_swing_foot_params(params, title, filename):
    ps = PlotStyler()
    ps.set_default_styling('bindings/pydairlib/cassie/data/')
    x, y, z = params_to_xyz_samples(params, 1.0, 100)
    ps.plot(x, z, data_label='$(\phi_{x}, z)$', linestyle='-')
    ps.plot(y, z, data_label='$(\phi_{y}, z)$', linestyle='--')
    plt.xlabel('$\phi$')
    plt.ylabel('$z (m)$')
    plt.title(title)
    plt.legend()
    ps.save_fig(filename)


def plot_params_from_file(params_filename):
    ps = PlotStyler()
    ps.set_default_styling()
    opt = SwingFootSplineOptimizer(0)
    p = opt.load_params(params_filename, opt.data_folder)
    params = np.array(get_default_params()) + p.value
    plot_swing_foot_params(
        params, 'Optimized Swing Foot Spline', 'swing_ft_optimized.png')
    plot_swing_foot_params(
        np.array(get_default_params()),
        'Unoptimized Swing Foot Spline', 'swing_ft_init.png')

def main():
    optimizer = SwingFootSplineOptimizer(1000)
    optimizer.learn_spline()


if __name__ == "__main__":
    plot_params_from_file('2022_06_08_23_1000')
