
from c3 import *
import numpy as np


def get_c3_options():
    c3_options = C3Options()

    # Q = 50 * np.diag(np.array([1500, 1000, 500, 500, 10, 10, 1, 1]))
    # R = 10 * np.eye(1)
    # G = np.diag(np.hstack((1e-2 * np.ones(4), 250 * np.ones(4),
    #                             50 * np.ones(4), 1e-2 * np.ones(1))))
    # U = 5 * np.diag(np.hstack((10 * np.ones(4), 1 * np.ones(4),
    #                                 1 * np.ones(4), 100000 * np.ones(1))))
    # c3_options.contact_model = "anitescu"

    contact_model = ContactModel.kStewartAndTrinkle
    Q = 50 * np.diag(np.array([1500, 1000, 500, 500, 10, 10, 1, 1]))
    R = 5 * np.eye(1)
    G = .005 * np.diag(np.hstack((1 * np.ones(4), 50000 * np.ones(4), 500000 * np.ones(2), 10000 * np.ones(6), 1 * np.ones(1))))
    U = 5 * np.diag(np.hstack((10 * np.ones(4), 1 * np.ones(4), 1 * np.ones(8), 100000 * np.ones(1))))
    c3_options.contact_model = "stewart_and_trinkle"

    c3_options.admm_iter = 5
    c3_options.rho = 0
    c3_options.rho_scale = 4
    c3_options.num_threads = 10
    c3_options.delta_option = 1

    c3_options.warm_start = 1
    c3_options.use_predicted_x0 = 0  # not necessary because we're not solving in a separate loop
    c3_options.end_on_qp_step = 0
    c3_options.use_robust_formulation = 0
    c3_options.solve_time_filter_alpha = 0.95
    c3_options.publish_frequency = 0
    c3_options.u_horizontal_limits = np.array([-100, 100])
    c3_options.u_vertical_limits = np.array([0, 0])
    c3_options.workspace_limits = [
        np.array([1, 0, 0, -0.5, 0.5])]  # unused
    c3_options.workspace_margins = 0.05
    c3_options.N = 10
    c3_options.gamma = 1.0
    c3_options.mu = [0.4, 0.4]
    c3_options.dt = 0.05
    c3_options.solve_dt = 0.0
    c3_options.num_friction_directions = 1
    c3_options.num_contacts = len(c3_options.mu)
    c3_options.Q = Q
    c3_options.R = R
    c3_options.G = G
    c3_options.U = U

    return c3_options