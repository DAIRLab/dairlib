import numpy as np
import sympy as sp
from scipy.special import factorial
class KoopmanModel:

    def __init__(self, dynamics, robot):
        '''
        This class represents a Single Rigid Body Koopman Model of a robot
        :param dynamics: function xdot = dynamics(x,u) representing the true dynamics of the robot
        :param robot: dict with the robot parameters 'mass', 'inertia', 'nx', 'nu' 'nc', 'mode'
        '''
        self.dynamics = dynamics
        self.robot = robot

    def sysID (self, x_samples, u_samples, x_order, u_order, basis_type):


    def CreateObservableBasisPolynomial(self, vars, deg):
        nx = vars.shape[0]
        N = factorial(nx + deg) / (factorial(nx) * factorial(deg))

        for i in range(deg):
            