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
        V_WA = gym.get_rigid_linear_velocity(env, self.bodyA_handle)
        V_WB = gym.get_rigid_linear_velocity(env, self.bodyB_handle)
        O_WA = gym.get_rigid_angular_velocity(env, self.bodyA_handle)
        O_WB = gym.get_rigid_angular_velocity(env, self.bodyB_handle)

        p_WP = X_WA.transform_point(self.p_AP)
        p_WQ = X_WB.transform_point(self.p_BQ)

        p_PQ_W = p_WQ - p_WP
        length_soft = p_PQ_W.length()
        r_PQ_W = p_PQ_W / length_soft

        p_PAo_W = X_WA.p - p_WP
        V_WP = V_WA + O_WA.cross(-p_PAo_W)

        p_QBo_W = X_WB.p - p_WQ
        V_WQ = V_WB + O_WB.cross(-p_QBo_W)

        v_PQ_W = V_WQ - V_WP
        length_dot = v_PQ_W.dot(r_PQ_W)

        f_AP_W = r_PQ_W * (self.stiffness * (length_soft - self.free_length))
        f_AP_W += r_PQ_W * self.damping * length_dot
        print(f_AP_W)

        gym.apply_body_forces(env, self.bodyA_handle, f_AP_W, p_WP, gymapi.ENV_SPACE)
        gym.apply_body_forces(env, self.bodyB_handle, -f_AP_W, p_WQ, gymapi.ENV_SPACE)

