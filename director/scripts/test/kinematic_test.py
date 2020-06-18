import lcm
import threading

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_robot_output

import director.applogic
import director.mainwindowapp
from director.debugVis import DebugData
import director.objectmodel as om
from pydairlib.common import FindResourceOrThrow

from pydrake.math import RigidTransform
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import numpy as np
import json
import sys
from collections import deque


class TestGui(QWidget):

    def __init__(self, parent = None):

        super(TestGui, self).__init__(parent)
        self.channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.lcm = lcm.LCM()
        self.checkBoxes = {}
        self.checkBoxesPrevState = {}
        self.data = None
        self.prev_loc = {}
        self.duration = {}
        self.shapes = {}
        self.plant = None
        self.resetBtn = None
        self.checkBoxArea = None
        self.reset = False

        # create the GUI
        self.setWindowTitle("Testing Testing")
        self.grid = QVBoxLayout()

        hbox = QHBoxLayout()

        # create the JSON directory reader
        hbox.addWidget(QLabel("Enter JSON file directory"))
        self.JSONInput = QLineEdit("./director/scripts/test/testJSON.json")
        hbox.addWidget(self.JSONInput)
        self.readJSON = QPushButton('Read JSON')
        self.readJSON.clicked.connect(self.readJSONFile)
        hbox.addWidget(self.readJSON)

        self.grid.addLayout(hbox)

        self.setLayout(self.grid)

        # Starting a new thread, since we need to block and wait for messages
        handle_thread = threading.Thread(target=self.handle_thread)
        handle_thread.start()

    def resetGUI(self):
        # delete checkboxes from GUI
        for i in reversed(range(self.checkBoxArea.count())):
            self.checkBoxArea.itemAt(i).widget().deleteLater()

        # reset GUI variables
        self.checkBoxes = {}
        self.checkBoxesPrevState = {}
        self.data = None
        self.prev_loc = {}
        self.duration = {}
        self.shapes = {}
        self.plant = None
        self.reset = False

    def readJSONFile(self):
        if (self.JSONInput.text != ""):
            self.json_file = self.JSONInput.text
            with open(self.json_file) as json_file:
                self.data = json.load(json_file)
            # fill the labels for each data with its name and add the reset button
            self.placeCheckBoxes()

            if (self.resetBtn == None):
                self.resetBtn = QPushButton('Reset')
                self.resetBtn.clicked.connect(self.deleteShapes)
                self.grid.addWidget(self.resetBtn)

    def deleteShapes(self):
        # vis.showText("hello", "Bobby", position=(100, 100))
        for shape in self.shapes.values():
            if (type(shape) == deque):
                for line in shape:
                    om.removeFromObjectModel(line)
            else:
                om.removeFromObjectModel(shape)

        self.reset = True

    def placeCheckBoxes(self):
        if (self.data):
            addToGUI = False
            if (self.checkBoxArea == None):
                self.checkBoxArea = QVBoxLayout()
                addToGUI = True
            for data in self.data['data']:
                jsonData = eval(str(data))
                addToList = False
                if (jsonData['name'] not in self.checkBoxes):
                        self.checkBoxes[jsonData['name']] = QCheckBox(jsonData['name'])
                        addToList = True
                self.checkBoxes[jsonData['name']].setChecked(True)
                self.checkBoxesPrevState[jsonData['name']] = True
                if (addToList == True):
                        self.checkBoxArea.addWidget(self.checkBoxes[jsonData['name']])

                if (addToGUI == True):
                        self.grid.addLayout(self.checkBoxArea)

    def checkBoxChecked(self, name):
        if (type(self.shapes[name]) == deque):
            for line in self.shapes[name]:
                line.setProperty('Visible', True)
        else:
            self.shapes[name].setProperty('Visible', True)

    def checkBoxNotChecked(self, name):
        if (type(self.shapes[name]) == deque):
            for line in self.shapes[name]:
                line.setProperty('Visible', False)
        else:
            self.shapes[name].setProperty('Visible', False)

    def handle_thread(self):
        subscription = self.lcm.subscribe(self.channel, self.state_handler)
        subscription.set_queue_capacity(1)

        print('Starting to handle LCM messages')
        while True:
            self.lcm.handle_timeout(100)

    def state_handler(self, channel, data):
        if (self.data):
            if (self.plant == None):
                # Create the plant (TODO: URDF name a JSON option)
                builder = pydrake.systems.framework.DiagramBuilder()
                self.plant, scene_graph = \
                    pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0)
                pydrake.multibody.parsing.Parser(self.plant).AddModelFromFile(
                FindResourceOrThrow(self.data['model_file']))

                weldBody = True

                # Fixed-base model (weld-body, or null, a JSON option)
                if ('weld-body' not in self.data):
                    weldBody = False

                if (weldBody == True):
                    self.plant.WeldFrames(self.plant.world_frame(),
                        self.plant.GetFrameByName("pelvis"), RigidTransform.Identity())
                self.plant.Finalize()
                self.context = self.plant.CreateDefaultContext()

            # TODO (for Michael): bind our more robust decoding mechanisms to python
            self.msg = dairlib.lcmt_robot_output.decode(data)
            self.plant.SetPositions(self.context, self.msg.position)
            self.plant.SetVelocities(self.context, self.msg.velocity)

            for data in self.data['data']:
                jsonData = eval(str(data))
                if (self.checkBoxes[jsonData['name']].isChecked() == True):
                    if (self.checkBoxesPrevState[jsonData['name']] == False):
                        self.checkBoxesPrevState[jsonData['name']] = True
                        self.checkBoxChecked(jsonData['name'])
                else:
                    if (self.checkBoxesPrevState[jsonData['name']] == True):
                        self.checkBoxesPrevState[jsonData['name']] = False
                        self.checkBoxNotChecked(jsonData['name'])

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
                    if (jsonData['name'] not in self.shapes):
                        self.shapes[jsonData['name']] = deque()

                    # check if there is any previously computed location
                    if (jsonData['name'] not in self.prev_loc):
                        self.prev_loc[jsonData['name']] = next_loc

                    # check if the duration has been initialized
                    if (jsonData['name'] not in self.duration):
                        self.duration[jsonData['name']] = self.msg.utime / 1000000

                    # visualize and trace line for 'history' seconds
                    if ((self.msg.utime / 1000000) - self.duration[jsonData['name']] <= jsonData['history']):
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
                    d = DebugData()
                    d.addSphere(next_loc, radius = jsonData['radius'])
                    # create a new point
                    if (jsonData['name'] not in self.shapes):
                        self.shapes[jsonData['name']] = vis.showPolyData(d.getPolyData(), 'sphere')
                        # set color and transparency of point
                        self.shapes[jsonData['name']].setProperty('Color', jsonData['color'])
                        self.shapes[jsonData['name']].setProperty('Alpha', jsonData['alpha'])
                    else:
                        # update the location of the last point
                        self.shapes[jsonData['name']].setPolyData(d.getPolyData())

        if (self.reset == True):
            self.resetGUI()

# Adding a widget but there's nothing in the widget (yet)
panel = TestGui()
panel.show()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)
