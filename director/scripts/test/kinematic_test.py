import lcm
import threading

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_robot_output

import director.applogic
import director.mainwindowapp
from director.debugVis import DebugData

from pydairlib.common import FindResourceOrThrow

from pydrake.math import RigidTransform
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import numpy as np
import json
import sys
from collections import deque
from functools import partial


class TestGui(QWidget):

    def __init__(self, parent = None):

        super(TestGui, self).__init__(parent)

        self.checkBoxes = {}

        # get names of all data
        self.json_file = sys.argv[3];
        with open(self.json_file) as json_file:
            self.data = json.load(json_file)

        # create the GUI
        self.setWindowTitle("Testing Testing")
        self.grid = QGridLayout()

        # fill the labels for each data with its name
        self.placeCheckBoxes()

        self.setLayout(self.grid)
        self.show()

        # Starting a new thread, since we need to block and wait for messages
        handle_thread = threading.Thread(target=self.handle_thread)
        handle_thread.start()

    def placeCheckBoxes(self):
        checkBoxes = []
        count = 0
        checkBoxes.append(QCheckBox("toe_point"))
        checkBoxes[count].stateChanged.connect(lambda: self.checkBoxChecked(checkBoxes[count]))
        self.grid.addWidget(checkBoxes[count])

        count += 1
        checkBoxes.append(QCheckBox("toe_line"))
        checkBoxes[count].stateChanged.connect(lambda: self.checkBoxChecked(checkBoxes[count]))
        self.grid.addWidget(checkBoxes[count])
        # for data in self.data['data']:
        #     jsonData = eval(str(data))
        #     checkBoxes.append(QCheckBox(jsonData['name']))
        #     checkBoxes[count].stateChanged.connect(self.checkBoxChecked("hi"))
        #     self.grid.addWidget(checkBoxes[count])
        #     count += 1

    def checkBoxChecked(self, checkbox):
        print(checkbox.text)
        # if (checkbox == "right_toe_line"):
        #     print("Hi")
        # else:
        #     print("Hello")

    def handle_thread(self):
        self.channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.lcm = lcm.LCM()
        self.prev_loc = {}
        self.duration = {}
        self.shapes = {}
        self.json_file = sys.argv[3];
        subscription = self.lcm.subscribe(self.channel, self.state_handler)
        subscription.set_queue_capacity(1)

        with open(self.json_file) as json_file:
            self.data = json.load(json_file)

        # Create the plant (TODO: URDF name a JSON option)
        builder = pydrake.systems.framework.DiagramBuilder()
        self.plant, scene_graph = \
            pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0)
        pydrake.multibody.parsing.Parser(self.plant).AddModelFromFile(
        FindResourceOrThrow(self.data['model_file']))

        weldBody = True

        # Fixed-base model (weld-body, or null, a JSON option)
        try:
            self.data['weld-body']
        except:
            weldBody = False

        if (weldBody):
            self.plant.WeldFrames(self.plant.world_frame(),
                self.plant.GetFrameByName("pelvis"), RigidTransform.Identity())
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        print('Starting to handle LCM messages')
        while True:
            self.lcm.handle_timeout(100)

    def state_handler(self, channel, data):
        # TODO (for Michael): bind our more robust decoding mechanisms to python
        msg = dairlib.lcmt_robot_output.decode(data)
        self.plant.SetPositions(self.context, msg.position)
        self.plant.SetVelocities(self.context, msg.velocity)

        for data in self.data['data']:
            jsonData = eval(str(data))

            # set the body and point on which to visualize the data
            pt_body = np.array(jsonData['point-on-body'])
            body_name = jsonData['body_part']

            # Use Drake's CalcPointsPositions to determine where that point is
            # in the world
            pt_world = self.plant.CalcPointsPositions(self.context,
                self.plant.GetFrameByName(body_name), pt_body,
                self.plant.world_frame())
            next_loc = pt_world.transpose()[0]

            # draw a continuous line
            if (jsonData['type'] == "line"):

                # check if this line has bee drawn or not
                try:
                    self.shapes[jsonData['name']]
                except:
                    self.shapes[jsonData['name']] = deque()

                # check if there is any previously computed location
                try:
                    self.prev_loc[jsonData['name']]
                except:
                    self.prev_loc[jsonData['name']] = next_loc

                # check if the duration has been initialized
                try:
                    self.duration[jsonData['name']]
                except:
                    self.duration[jsonData['name']] = msg.utime / 1000000

                # visualize and trace line for 'history' seconds
                if ((msg.utime / 1000000) - self.duration[jsonData['name']] <= jsonData['history']):
                    # add new line
                    d = DebugData()
                    d.addLine(self.prev_loc[jsonData['name']], next_loc, radius = jsonData['thickness'])
                    line = vis.showPolyData(d.getPolyData(), 'line')

                    # set color and transparency of line
                    line.setProperty('Color', jsonData['color'])
                    line.setProperty('Alpha', jsonData['alpha'])

                    # add line to the history of current lines drawn
                    self.shapes[jsonData['name']].append(line);
                else:
                    # reset the points of the last placed line
                    d = DebugData()
                    d.addLine(self.prev_loc[jsonData['name']], next_loc, radius = jsonData['thickness'])
                    lastLine = self.shapes[jsonData['name']].popleft()
                    lastLine.setPolyData(d.getPolyData())
                    self.shapes[jsonData['name']].append(lastLine)

                self.prev_loc[jsonData['name']] = next_loc

            # draw a point
            elif (jsonData['type'] == "point"):

                # check if the point has already been drawn
                try:
                    self.shapes[jsonData['name']]
                except:
                    self.shapes[jsonData['name']] = None

                d = DebugData()
                d.addSphere(next_loc, radius = jsonData['radius'])
                # create a new point
                if (self.shapes[jsonData['name']] == None):
                    self.shapes[jsonData['name']] = vis.showPolyData(d.getPolyData(), 'sphere')
                    # set color and transparency of point
                    self.shapes[jsonData['name']].setProperty('Color', jsonData['color'])
                    self.shapes[jsonData['name']].setProperty('Alpha', jsonData['alpha'])
                else:
                    # update the location of the last point
                    self.shapes[jsonData['name']].setPolyData(d.getPolyData())

# Adding a widget but there's nothing in the widget (yet)
panel = TestGui()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)

import director.openscope as scope
import subprocess
view = applogic.getMainWindow()
applogic.addShortcut(view, 'Ctrl+I', scope.startSignalScope)
