import sys
import time
import math
import lcm

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_cassie_pd_config
import dairlib.lcmt_cassie_state

import director.applogic
import director.mainwindowapp


#Default values
joint_names = [
  "L_HIP_ROLL",
  "R_HIP_ROLL",
  "L_HIP_YAW",
  "R_HIP_YAW",
  "L_HIP_PITCH",
  "R_HIP_PITCH",
  "L_KNEE",
  "R_KNEE",
  "L_FOOT",
  "R_FOOT"]
joint_default = [0,0,0,0,0,0,0,0,0,0]
kp_default = [0,0,0,0,0,0,0,0,0,0]
kd_default = [0,0,0,0,0,0,0,0,0,0]


class ControllerGui(QWidget):

    def __init__(self, parent = None):

        super(ControllerGui, self).__init__(parent)

        self.lc = lcm.LCM()
        subscription = self.lc.subscribe("CASSIE_STATE", self.state_handler)
        subscription.set_queue_capacity(1)

        labels = []
        self.values = []
        self.ledits = []
        for name in joint_names:
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

        grid.addWidget(self.publish_button, len(joint_names) + 2, 0)
        grid.addWidget(self.setState_button, len(joint_names) + 2, 3)

        #Initializing the text boxes to the initial values
        self.initialize_default()

        self.connect(self.publish_button, SIGNAL("clicked()"), self.publish_clicked)

        self.connect(self.setState_button, SIGNAL("clicked()"), self.setState_clicked)

        self.setLayout(grid)
        #self.setWindowTitle("Controller GUI")
        #self.resize(400, 300)

    def value_change(self):
        print("Asdf")
        for idx, ledit in enumerate(self.ledits):
            self.values[idx] = self.ledits[idx].value

    #Initial defafult text values
    def initialize_default(self):
        for idx, name in enumerate(joint_names):
            self.ledits[idx].setValue(joint_default[idx])
            self.ledits[idx + len(joint_names)].setValue(kp_default[idx])
            self.ledits[idx + 2*len(joint_names)].setValue(kd_default[idx])


    #Storing in a file once the move button is clicked
    def publish_clicked(self):
        msg = dairlib.lcmt_cassie_pd_config()
        msg.timestamp = int(time.time() * 1e6)
        msg.num_joints = 10
        msg.joint_names = joint_names
        msg.desired_position = self.values[0:len(joint_names)]
        msg.desired_position = [x for x in msg.desired_position]
        msg.desired_velocity = [0,0,0,0,0,0,0,0,0,0]
        msg.kp = self.values[len(joint_names):2*len(joint_names)]
        msg.kd = self.values[2*len(joint_names):3*len(joint_names)]

        self.lc.publish("PD_CONFIG", msg.encode())

    def setState_clicked(self):
        self.lc.handle_timeout(100)
    def state_handler(self, channel, data):
        msg = dairlib.lcmt_cassie_state.decode(data)
        for joint in msg.joint_names:
            idx = joint_names.index(joint)
            self.ledits[idx].setValue(msg.position[idx])
            self.values[idx] = msg.position[idx]


panel = ControllerGui()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)

import director.openscope as scope
import subprocess
view = applogic.getMainWindow()
applogic.addShortcut(view, 'Ctrl+I', scope.startSignalScope)