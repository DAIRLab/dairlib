import numpy as np
from isaacgym import gymapi, gymutil
from math import sqrt


class IsaacSpringConstraint:

    def __init__(self, bodyA_handle, bodyB_handle, p_AP, p_BQ, free_length, stiffness, damping):
        self.bodyA_handle = bodyA_handle
        self.bodyB_handle = bodyB_handle
        self.p_AP = p_AP
        self.p_BQ = p_BQ
        self.free_length = free_length
        self.stiffness = stiffness
        self.damping = damping

    def CalcAndAddForceContribution(self, gym, env):

        X_WA = gym.get_rigid_transform(env, self.bodyA_handle)
        X_WB = gym.get_rigid_transform(env, self.bodyB_handle)

        p_WP = X_WA.transform_point(self.p_AP)
        p_WQ = X_WB.transform_point(self.p_BQ)

        p_PQ_W = p_WQ - p_WP
        length_soft = p_PQ_W.length()
        r_PQ_W = p_PQ_W / length_soft

        # TODO compute length_dot
        length_dot = 0
        # import pdb; pdb.set_trace()
        f_AP_W = r_PQ_W * (self.stiffness * (length_soft - self.free_length))
        f_AP_W += r_PQ_W * self.damping * length_dot

        gym.apply_body_forces(env, self.bodyA_handle, f_AP_W, p_WP, gymapi.ENV_SPACE)
        gym.apply_body_forces(env, self.bodyB_handle, -f_AP_W, p_WQ, gymapi.ENV_SPACE)

