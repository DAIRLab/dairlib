import numpy as np

import dairlib
from pydrake.multibody.plant import *
from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydrake.multibody.tree import MultibodyForces
from pydairlib.multibody import *
import cassie_plotting_utils as cassie_plots
from scipy.spatial.transform import Rotation as R

def quat_dot(quat, omega):
    """
    Borrowed from quadrotor sim from MEAM 620
    Parameters:
        quat, [i,j,k,w]
        omega, angular velocity of body in body axes

    Returns
        duat_dot, [i,j,k,w]

    """
    # Adapted from "Quaternions And Dynamics" by Basile Graf.
    (q0, q1, q2, q3) = (quat[0], quat[1], quat[2], quat[3])
    G = np.array([[ q3,  q2, -q1, -q0],
                  [-q2,  q3,  q0, -q1],
                  [ q1, -q0,  q3, -q2]])
    quat_dot = 0.5 * G.T @ omega
    # Augment to maintain unit quaternion.
    quat_err = np.sum(quat**2) - 1
    quat_err_grad = 2 * quat
    quat_dot = quat_dot - quat_err * quat_err_grad
    return quat_dot

class SpringCompensation():
    def __init__(self):
        self.dt = 8e-5
        self.plant, self.context = cassie_plots.make_plant_and_context(
            floating_base=True, springs=True)
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()
        self.nx = self.nq + self.nv
        self.B = self.plant.MakeActuationMatrix()

    def sample_simulate_forward(self, q, v, u, n_samples):
        x = np.hstack((q, v))
        self.plant.SetPositionsAndVelocities(self.context, x)

        samples = np.zeros((n_samples, self.nx))
        # t_samples = np.zeros((n_samples, 1))
        t_samples = 0.01 * np.random.sample(n_samples)
        t_samples = np.sort(t_samples)
        M = self.plant.CalcMassMatrix(self.context)
        c = self.plant.CalcBiasTerm(self.context)
        f = MultibodyForces(self.plant)
        self.plant.CalcForceElementsContribution(self.context, f)
        g = self.plant.CalcGravityGeneralizedForces(self.context)
        M_inv = np.linalg.inv(M)
        vdot = M_inv @ (-c + f.generalized_forces() + g + self.B @ u)
        # import pdb; pdb.set_trace()
        # t = 0.01 * np.random.sample()
        for i in range(n_samples):
            x_next = self.simulate_forward(x, vdot, t_samples[i])
            samples[i] = x_next
            # t_samples[i] = t
        return samples, t_samples

    def simulate_forward(self, x, vdot, t):
        t0 = 0
        q = np.copy(x[:self.nq])
        v = np.copy(x[-self.nv:])
        while t0 < t:
            q[:4] = q[:4] + self.dt * quat_dot(q[:4], v[:3])
            q[4:] = q[4:] + self.dt * v[3:]
            v = v + self.dt * vdot
            t0 += self.dt
        v[12] = 0
        v[20] = 0
        x_next = np.hstack((q, v))

        return x_next

