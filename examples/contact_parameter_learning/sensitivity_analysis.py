import numpy as np

def sensitivity_analysis(sim, loss_weights, optimal_params, params_range):
    sim.init_params(optimal_params)
    