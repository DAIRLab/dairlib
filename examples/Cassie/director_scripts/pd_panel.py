import sys
import time
import math
import lcm

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_pd_config
import dairlib.lcmt_robot_output

import director.applogic
import director.mainwindowapp


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
joint_default = [-0.01,.01,0,0,0.55,0.55,-1.5,-1.5,-1.8,-1.8]
kp_default = [i for i in [80,80,50,50,50,50,50,50,10,10]]
kd_default = [i for i in [1,1,1,1,1,1,2,2,1,1]]


class ControllerGui(QWidget):

    def __init__(self, parent = None):
        if 'pd_panel_state_channel' in globals():
            channel = pd_panel_state_channel
        else:
            channel = "CASSIE_STATE_SIMULATION"
        # TODO: We might need to change to NETWORK_CASSIE_STATE_DISPATCHER if the above code is not working on real robot

        super(ControllerGui, self).__init__(parent)

        self.lc = lcm.LCM()
        subscription = self.lc.subscribe(channel, self.state_handler)
        subscription.set_queue_capacity(1)

        print(channel)

        labels = []
        self.values = []
        self.ledits = []
        for name in position_names:
            labels.append(QLabel(name))
            self.values.append(0)
            self.ledits.append(QDoubleSpinBox())
            self.values.append(0)
            self.ledits.append(QDoubleSpinBox())
            self.values.append(0)
            self.ledits.append(QDoubleSpinBox())

        for ledit in self.ledits:
            ledit.setFixedSize(100, 25)


        #Initializing widgets

        self.publish_button = QPushButton('Publish')

        self.setState_button = QPushButton('Set from State')

        self.publishHeight_button = QPushButton('Set height')

        #Grid and widget locations
        self.widget = QWidget()

        self.widget.setLayout(QGridLayout())

        grid = QGridLayout()

        grid.addWidget(QLabel("Desired Position"), 0, 1)
        grid.addWidget(QLabel("KP"), 0, 3)
        grid.addWidget(QLabel("KD"), 0, 5)

        for idx, name in enumerate(joint_names):
            grid.addWidget(labels[idx], idx+1, 0)
            grid.addWidget(self.ledits[idx], idx+1, 1)
            self.ledits[idx].setMinimum(-3.14)
            self.ledits[idx].setMaximum(3.14)
            self.ledits[idx].setSingleStep(.01)

            grid.addWidget(self.ledits[idx + len(joint_names)], idx+1, 3)

            self.ledits[idx + len(joint_names)].setMinimum(-100)
            self.ledits[idx + len(joint_names)].setMaximum(100)
            self.ledits[idx + len(joint_names)].setSingleStep(1)

            grid.addWidget(self.ledits[idx + 2*len(joint_names)], idx+1, 5)
            self.ledits[idx + 2*len(joint_names)].setMinimum(-100)
            self.ledits[idx + 2*len(joint_names)].setMaximum(100)
            self.ledits[idx + 2*len(joint_names)].setSingleStep(1)

        for ledit in self.ledits:
            self.connect(ledit, SIGNAL("editingFinished()"), self.value_change)

        grid.addWidget(self.publish_button, len(joint_names) + 4, 0)
        grid.addWidget(self.setState_button, len(joint_names) + 4, 3)
        grid.addWidget(self.publishHeight_button, len(joint_names) + 4, 6)

        # Box for ramp up time
        self.ramp_up_time_box = QDoubleSpinBox();
        grid.addWidget(QLabel("Ramp up time"), 11, 0)
        grid.addWidget(self.ramp_up_time_box, 11, 1)
        self.ramp_up_time = 0.5

        self.target_height_box = QDoubleSpinBox();
        grid.addWidget(QLabel("Target height"), 13, 0)
        grid.addWidget(self.target_height_box, 13, 1)
        self.target_height_box.value = 0.9
        self.target_height = 0.9

        #Initializing the text boxes to the initial values
        self.initialize_default()
        for idx, ledit in enumerate(self.ledits):
            self.values[idx] = self.ledits[idx].value
        

        self.connect(self.publish_button, SIGNAL("clicked()"), self.publish_clicked)

        self.connect(self.setState_button, SIGNAL("clicked()"), self.setState_clicked)

        self.connect(self.publishHeight_button, SIGNAL("clicked()"), self.publishHeight_clicked)

        self.setLayout(grid)
        #self.setWindowTitle("Controller GUI")
        #self.resize(400, 300)

        # previous desired positions set by the user
        self.prev_pos_ = []

        # previous kp and kd gain set by the user
        self.kp_ = [0,0,0,0,0,0,0,0,0,0]
        self.kd_ = [0,0,0,0,0,0,0,0,0,0]

    def value_change(self):
        for idx, ledit in enumerate(self.ledits):
            self.values[idx] = self.ledits[idx].value
        self.ramp_up_time = self.ramp_up_time_box.value
        print('value changed')

    #Initial defafult text values
    def initialize_default(self):
        for idx, name in enumerate(joint_names):
            self.ledits[idx].setValue(joint_default[idx])
            self.ledits[idx + len(joint_names)].setValue(kp_default[idx])
            self.ledits[idx + 2*len(joint_names)].setValue(kd_default[idx])
        self.ramp_up_time_box.setValue(0.5)

    def publishHeight_clicked(self):
        self.target_height = self.target_height_box.value
        height_msg = dairlib.lcmt_target_standing_height()
        height_msg.timestamp = int(time.time() * 1e6)
        height_msg.target_height = self.target_height
        self.lc.publish("TARGET_HEIGHT", height_msg.encode())

    #Storing in a file once the move button is clicked  
    def publish_clicked(self):
        msg = dairlib.lcmt_pd_config()
        msg.num_joints = 10
        msg.joint_names = joint_names
        msg.desired_velocity = [0,0,0,0,0,0,0,0,0,0]

        first_loop = False
        if len(self.prev_pos_) == 0:
            first_loop = True
            msg.desired_position = self.values[0:len(joint_names)]
            print(msg.desired_position)
            # msg.desired_position = [x for x in msg.desired_position]

        for i in range(100):
            # ramp up the gains for 5 seconds
            msg.timestamp = int(time.time() * 1e6)
            msg.kp = [b + i / 99.0 * (a - b) for a, b in zip(self.values[len(joint_names):2*len(joint_names)], self.kp_)]
            msg.kd = [b + i / 99.0 * (a - b) for a, b in zip(self.values[2*len(joint_names):3*len(joint_names)], self.kd_)]

            # ramp up the desired positions for 5 seconds
            if not first_loop:
                msg.desired_position = [b + i / 99.0 * (a - b) for a, b in zip(self.values[0:len(joint_names)], self.prev_pos_)]

            self.lc.publish("PD_CONFIG", msg.encode())
            time.sleep(self.ramp_up_time / 100.0)
        # store previous kp kd gains
        self.kp_ = msg.kp
        self.kd_ = msg.kd
        self.prev_pos_ = msg.desired_position


    def setState_clicked(self):
        self.lc.handle_timeout(100)
        self.lc.handle_timeout(100) #twice to clear the buffer
    def state_handler(self, channel, data):
        msg = dairlib.lcmt_robot_output.decode(data)
        for idx_msg, joint in enumerate(msg.position_names):
            if joint in position_names:
                idx = position_names.index(joint)
                self.ledits[idx].setValue(msg.position[idx_msg])
                self.values[idx] = msg.position[idx_msg]
        print('message handled')
        print(msg.position[6])


panel = ControllerGui()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)

import director.openscope as scope
import subprocess
view = applogic.getMainWindow()
applogic.addShortcut(view, 'Ctrl+I', scope.startSignalScope)
