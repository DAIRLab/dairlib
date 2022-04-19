import numpy as np

class ekf_robot_state_history:
    def __init__(self):
        self.t = []
        self.R_prop = []        # Orientation
        self.R_corr = []
        self.v_prop = []        # Linear velocity
        self.v_corr = []
        self.r_prop = []        # Position
        self.r_corr = []
        self.theta_prop = []    # Imu bias
        self.theta_corr = []
        self.contact_prop = []  # Contact State
        self.contact_corr = []

    def append(self, msg):
        import pdb; pdb.set_trace()
        self.t.append(msg.utime * 1e-6)
        X_prop = np.array(msg.X_prop)
        X_corr = np.array(msg.X_corr)
        if not X_prop[0]:
            return
        self.R_prop.append(X_prop[:, :3])
        self.R_corr.append(X_corr[:, :3])
        self.v_prop.append(X_prop[:, 3])
        self.v_corr.append(X_corr[:, 3])
        self.r_prop.append(X_prop[:, 4])
        self.r_corr.append(X_corr[:, 4])
        self.theta_prop.append(msg.theta_prop)
        self.theta_corr.append(msg.theta_corr)

    def convert(self):
        self.theta_prop = np.array(self.theta_prop)
        self.theta_corr = np.array(self.theta_corr)
