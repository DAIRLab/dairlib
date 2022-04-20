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
        self.Theta_prop = []    # Imu bias
        self.Theta_corr = []
        self.contact_prop = []  # Contact State
        self.contact_corr = []

    def append(self, msg):
        self.t.append(msg.utime * 1e-6)
        X_prop = np.array(msg.X_prop)
        import pdb; pdb.set_trace()
        X_corr = np.array(msg.X_corr)
        self.R_prop.append(X_prop[:, :3])
        self.R_corr.append(X_corr[:, :3])
        self.v_prop.append(X_prop[:, 3])
        self.v_corr.append(X_corr[:, 3])
        self.r_prop.append(X_prop[:, 4])
        self.r_corr.append(X_corr[:, 4])
        self.Theta_prop.append(msg.Theta_prop)
        self.Theta_corr.append(msg.Theta_corr)

    def convert(self):
        self.Theta_prop = np.array(self.Theta_prop)
        self.Theta_corr = np.array(self.Theta_corr)
