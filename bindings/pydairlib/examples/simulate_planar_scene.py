import pydairlib.examples.planar_scene_simulator as sim
import numpy as np

if __name__ == "__main__":
    T = 2
    sim_dt = 1e-3
    output_dt = 1e-3
    # x0 is (x0,z0,theta0,x1,z1,theta1,...xdot0,zdot0,thetadot0,....)
    # ordered circles, ellipses, capsules
    q0 = np.array([2, 3, 0, -2, 5, 0, 3, 6, 0, 0, 5, 0])
    v0 = np.zeros(12)
    x0 = np.concatenate((q0, v0))
    circles = np.array([[.2, 1, .5], [.4, 1, .5]])
    ellipses = np.array([[1, .5, 2, .5]])
    capsules = np.array([[.5, 1, 2, .3]])
    t, x, F = sim.SimulateScene(T=T, sim_dt=sim_dt, output_dt=output_dt,
                                circles=circles, ellipses=ellipses,
                                capsules=capsules, x0=x0, visualize=True)
