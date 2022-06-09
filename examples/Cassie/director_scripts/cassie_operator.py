import time
import lcm

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_pd_config
import dairlib.lcmt_robot_output

#Default values
joint_names = [
    "hip_roll_left_motor",
    "hip_roll_right_motor",
    "hip_yaw_left_motor",
    "hip_yaw_right_motor",
    "hip_pitch_left_motor",
    "hip_pitch_right_motor",
    "knee_left_motor",
    "knee_right_motor",
    "toe_left_motor",
    "toe_right_motor"]

position_names = [
    "hip_roll_left",
    "hip_roll_right",
    "hip_yaw_left",
    "hip_yaw_right",
    "hip_pitch_left",
    "hip_pitch_right",
    "knee_left",
    "knee_right",
    "toe_left",
    "toe_right"]


# Set of gains with which COM is within support polygon when we lower the hoist
joint_default = [-0.01, .01, 0, 0, 0.55, 0.55, -1.5, -1.5, -1.8, -1.8]
kp_default = [80, 80, 50, 50, 50, 50, 50, 50, 10, 10]
kd_default = [1, 1, 1, 1, 1, 1, 2, 2, 1, 1]


class ControllerGui(QWidget):

    def __init__(self, parent = None):

        super(ControllerGui, self).__init__(parent)
        self.lc = lcm.LCM()

        # Initializing widgets
        self.publish_button = QPushButton('Publish')
        self.publish_height_button = QPushButton('Set height')

        # Grid and widget locations
        self.widget = QWidget()
        self.widget.setLayout(QGridLayout())
        grid = QGridLayout()

        grid.addWidget(self.publish_button, 0, 1)
        grid.addWidget(self.publish_height_button, 2, 1)

        self.ramp_up_time = 0.5

        self.target_height_box = QDoubleSpinBox();
        grid.addWidget(QLabel("Target height"), 3, 0)
        grid.addWidget(self.target_height_box, 3, 1)
        self.target_height_box.value = 0.9
        self.target_height = 0.9

        self.connect(self.publish_button, SIGNAL("clicked()"), self.publish_pd)
        self.connect(self.publish_height_button,
                     SIGNAL("clicked()"), self.publish_height_clicked)
        self.setLayout(grid)
        self.resize(100, 50)

    def publish_height_clicked(self):
        self.target_height = self.target_height_box.value
        height_msg = dairlib.lcmt_target_standing_height()
        height_msg.timestamp = int(time.time() * 1e6)
        height_msg.target_height = self.target_height
        self.lc.publish("TARGET_HEIGHT", height_msg.encode())

    def publish_pd(self):
        msg = dairlib.lcmt_pd_config()
        msg.num_joints = 10
        msg.joint_names = joint_names
        msg.desired_position = joint_default
        msg.desired_velocity = [0,0,0,0,0,0,0,0,0,0]

        for i in range(100):
            # ramp up the gains for 5 seconds
            msg.timestamp = int(time.time() * 1e6)
            msg.kp = [kp_default[j] * i / 100 for j in range(len(joint_names))]
            msg.kd = [kd_default[j] * i / 100 for j in range(len(joint_names))]
            self.lc.publish("PD_CONFIG", msg.encode())
            time.sleep(self.ramp_up_time / 100.0)

panel = ControllerGui()
app.addWidgetToDock(panel, QtCore.Qt.BottomDockWidgetArea)