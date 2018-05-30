import sys
import time
import math
import lcm
from PyQt4.QtGui import *
from PyQt4.QtCore import *

# this import line...cannot be right, but it's needed right now to work
import dair_lab.utility.applications.cassie.lcmtypes.drake.lcmt_cassie_pd_config

#Default values
joint_names = ["L_HIP_ROLL", "L_HIP_YAW", "L_HIP_PITCH", "L_KNEE", "L_FOOT",
      "R_HIP_ROLL", "R_HIP_YAW", "R_HIP_PITCH", "R_KNEE", "R_FOOT"]
joint_default = [0,0,0,0,0,0,0,0,0,0]
kp_default = [0,0,0,0,0,0,0,0,0,0]
kd_default = [0,0,0,0,0,0,0,0,0,0]


class ControllerGui(QDialog):

    def __init__(self, parent = None):

        super(ControllerGui, self).__init__(parent)


        labels = []
        self.sliders = []
        self.values = []
        self.ledits = []
        for name in joint_names:
            labels.append(QLabel(name))
            self.sliders.append(QSlider(Qt.Horizontal))
            self.values.append(0)
            self.ledits.append(QLineEdit())
            self.sliders.append(QSlider(Qt.Horizontal))
            self.values.append(0)
            self.ledits.append(QLineEdit())
            self.sliders.append(QSlider(Qt.Horizontal))
            self.values.append(0)
            self.ledits.append(QLineEdit())

        for idx, name in enumerate(joint_names):
            self.sliders[idx].setMinimum(-180)
            self.sliders[idx].setMaximum(180)
            self.sliders[idx + len(joint_names)].setMinimum(0)
            self.sliders[idx + len(joint_names)].setMaximum(200)
            self.sliders[idx + 2*len(joint_names)].setMinimum(0)
            self.sliders[idx + 2*len(joint_names)].setMaximum(20)

        for ledit in self.ledits:
            ledit.setFixedSize(45, 25)

        #Initializing widgets

        self.publish_button = QPushButton('Publish')

        #Grid and widget locations

        grid = QGridLayout()

        grid.addWidget(QLabel("Desired Position"), 0, 1)
        grid.addWidget(QLabel("KP"), 0, 3)
        grid.addWidget(QLabel("KD"), 0, 5)

        for idx, name in enumerate(joint_names):
            grid.addWidget(labels[idx], idx+1, 0)
            grid.addWidget(self.sliders[idx], idx+1, 1)
            grid.addWidget(self.ledits[idx], idx+1, 2)

            grid.addWidget(self.sliders[idx + len(joint_names)], idx+1, 3)
            grid.addWidget(self.ledits[idx + len(joint_names)], idx+1, 4)

            grid.addWidget(self.sliders[idx + 2*len(joint_names)], idx+1, 5)
            grid.addWidget(self.ledits[idx + 2*len(joint_names)], idx+1, 6)

        for slider in self.sliders:
            slider.valueChanged.connect(self.slider_change)

        for ledit in self.ledits:
            self.connect(ledit, SIGNAL("editingFinished()"), self.text_change)

        grid.addWidget(self.publish_button, len(joint_names) + 2, 3)

        #Initializing the text boxes to the initial values
        self.initialize_default()

        self.connect(self.publish_button, SIGNAL("clicked()"), self.publish_clicked)

        self.setLayout(grid)
        self.setWindowTitle("Controller GUI")
        self.resize(600, 400)

    def slider_change(self):

        for idx,slider in enumerate(self.sliders):
            self.values[idx] = float(self.sliders[idx].value())
            self.ledits[idx].setText(str(self.values[idx]))

    def text_change(self):

        for idx, ledit in enumerate(self.ledits):
            self.values[idx] = float(self.ledits[idx].text())
            self.sliders[idx].setValue(self.values[idx])

    #Initial defafult text values
    def initialize_default(self):
        for idx, name in enumerate(joint_names):
            self.ledits[idx].setText(str(joint_default[idx]))
            self.ledits[idx + len(joint_names)].setText(str(kp_default[idx]))
            self.ledits[idx + 2*len(joint_names)].setText(str(kd_default[idx]))


    #Storing in a file once the move button is clicked
    def publish_clicked(self):
        msg = dair_lab.utility.applications.cassie.lcmtypes.drake.lcmt_cassie_pd_config()
        msg.timestamp = int(time.time() * 1e6)
        msg.num_joints = 10
        msg.joint_names = joint_names
        msg.desired_position = self.values[0:len(joint_names)]
        msg.desired_position = [x*math.pi/180 for x in msg.desired_position]
        msg.desired_velocity = [0,0,0,0,0,0,0,0,0,0]
        msg.kp = self.values[len(joint_names):2*len(joint_names)]
        msg.kd = self.values[2*len(joint_names):3*len(joint_names)]

        lc = lcm.LCM()
        lc.publish("PD_CONFIG", msg.encode())

def main():
    app = QApplication(sys.argv)
    cgui = ControllerGui()
    cgui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
