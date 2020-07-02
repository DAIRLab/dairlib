import lcm
import threading, queue

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_robot_output
from director import visualization as vis
import director.applogic
import director.mainwindowapp
from director.debugVis import DebugData
import director.objectmodel as om
from pydairlib.common import FindResourceOrThrow

from pydrake.math import RigidTransform, RotationMatrix
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import numpy as np
import json
import sys
from collections import deque

class VisualizationGui(QWidget):

    def __init__(self, parent = None):
        super(VisualizationGui, self).__init__(parent)
        self.channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.lcm = lcm.LCM()

        # GUI attributes
        self.checkBoxes = {}
        self.checkBoxesPrevState = {}
        self.resetBtn = None
        self.clearBtn = None
        self.checkBoxArea = None
        self.reset = False
        self.delete = False
        self.clear = False
        self.ready = False

        # JSON attributes
        self.data = None
        self.modelFile = None
        self.weldBody = False
        self.shapes = {}
        self.plant = None

        # add lcm channel
        subscription = self.lcm.subscribe(self.channel, self.state_handler)
        subscription.set_queue_capacity(1)

        # create the GUI
        self.setWindowTitle("Testing Testing")
        self.vbox = QVBoxLayout()
        hbox = QHBoxLayout()

        # create the JSON directory reader
        hbox.addWidget(QLabel("Enter JSON file directory"))
        self.JSONInput = QLineEdit("./director/scripts/test/testJSON.json")
        hbox.addWidget(self.JSONInput)
        self.readJSON = QPushButton('Read JSON')
        self.readJSON.clicked.connect(self.readJSONFile)
        hbox.addWidget(self.readJSON)
        self.vbox.addLayout(hbox)

        # Starting a new thread, since we need to block and wait for messages
        handle_thread = threading.Thread(target=self.handle_thread)
        handle_thread.start()

    def resetGUI(self):
        '''
        Function for reseting the GUI to its initial state
        '''

        # delete checkboxes from GUI
        for i in reversed(range(self.checkBoxArea.count())):
            self.checkBoxArea.itemAt(i).widget().deleteLater()

        # reset GUI variables
        self.checkBoxes = {}
        self.checkBoxesPrevState = {}
        self.shapes = {}
        self.reset = False
        self.delete = False
        self.clear = False
        self.ready = False

        # reset JSON variables
        self.data = None
        self.modelFile = None
        self.weldBody = False
        self.shapes = {}
        self.plant = None

    def readJSONFile(self):
        '''
        Function for reading JSON input file and populating the JSON
        and GUI attributes
        '''

        if (self.JSONInput.text != ""):
            self.json_file = self.JSONInput.text
            with open(self.json_file) as json_file:
                self.data = json.load(json_file)

            self.ready = True
            self.modelFile = self.data['model_file']
            if ('weld-body' in self.data):
                self.weldBody = True

            for data in self.data['data']:
                newObject = ObjectToDraw(data)
                if (newObject.name not in self.shapes):
                    self.shapes[newObject.name] = newObject
                else:
                    self.shapes[newObject.name].update(newObject)

            # fill the chackbox for each data with its name
            self.placeCheckBoxes()

            # add "reset" and "clear history" buttons
            if (self.resetBtn == None):
                self.resetBtn = QPushButton('Reset')
                self.resetBtn.clicked.connect(self.deleteShapes)
                self.vbox.addWidget(self.resetBtn)

            if (self.clearBtn == None):
                self.clearBtn = QPushButton('Clear History')
                self.clearBtn.clicked.connect(self.clearHistory)
                self.vbox.addWidget(self.clearBtn)

    def deleteShapes(self):
        '''
        Function for setting the flag for deleting all shapes currently present
        '''
        if (self.delete == False):
            self.delete = True

    def clearHistory(self):
        '''
        Function for setting the flag for clearing the history of any line present
        '''
        if (self.clear == False):
            self.clear = True

    def placeCheckBoxes(self):
        '''
        Function for placing the chackboxes of the GUI. Each checkbox corresponds
        to a shape/object that has been drawn with the corresponding color and
        shape
        '''
        if (self.ready == True):
            addToGUI = False
            if (self.checkBoxArea == None):
                self.checkBoxArea = QVBoxLayout()
                addToGUI = True
            for name in self.shapes:
                dotOrLine = " •"
                if (self.shapes[name].type == "point"):
                    dotOrLine = " •"
                else:
                    dotOrLine = " ---"

                addToList = False
                if (name not in self.checkBoxes):
                    self.checkBoxes[name] = QCheckBox(name + dotOrLine)
                    color = self.shapes[name].color
                    self.checkBoxes[name].setStyleSheet("color: rgb("+str(color[0] * 255)+", "+str(color[1] * 255)+", "+str(color[2] * 255)+")")
                    addToList = True
                self.checkBoxes[name].setChecked(True)
                self.checkBoxesPrevState[name] = True
                if (addToList == True):
                        self.checkBoxArea.addWidget(self.checkBoxes[name])
                if (addToGUI == True):
                        self.vbox.addLayout(self.checkBoxArea)

    def checkBoxChecked(self, name):
        '''
        Function for showing a shape when its corresponding checkbox is checked
        '''
        if (self.shapes[name].type == "line"):
            for line in self.shapes[name].object:
                line.setProperty('Visible', True)
        else:
            self.shapes[name].object.setProperty('Visible', True)

    def checkBoxNotChecked(self, name):
        '''
        Function for hiding a shape when its corresponding checkbox is unchecked
        '''
        if (self.shapes[name].type == "line"):
            for line in self.shapes[name].object:
                line.setProperty('Visible', False)
        else:
            self.shapes[name].object.setProperty('Visible', False)

    def distance(self, pt1, pt2):
        '''
        Function for computing distance between 2 given 3D points
        '''
        sum = 0
        for i in range(len(pt1)):
            sum += pow(pt2[i] - pt1[i], 2)

        return math.sqrt(sum)

    def handle_thread(self):
        '''
        Function for running the main thread
        '''
        print('Starting to handle LCM messages')
        while True:
            # read lcm messages
            self.lcm.handle_timeout(100)

            # handle delete, clear, and reset flags
            if (self.delete == True):
                for shape in self.shapes.values():
                    if (shape.type == "line"):
                        for line in shape.object:
                            om.removeFromObjectModel(line)
                    else:
                        om.removeFromObjectModel(shape.object)

                self.delete = False
                self.reset = True

            if (self.clear == True):
                for shape in self.shapes.values():
                    if (shape.type == "line"):
                        for line in shape.object:
                            om.removeFromObjectModel(line)
                    shape.object = deque()
                self.clear = False

    # The following is a test function for handling messages from different lcm
    # channels

    # def abstract_handler(self, channel, data):
    #     decoder = getattr(dairlib, self.abstract_type)
    #     msg = decoder.decode(data)
    #     field = getattr(msg, "tracking_data")
    #     # pt = np.array([field[self.abstract_x_index],
    #     #                field[self.abstract_y_index],
    #     #                field[self.abstract_z_index]])
    #
    #     a = None
    #     for el in field:
    #         if (el.name == "cp_traj"):
    #             a = el.y
    #             print(a)
    #             break

    def state_handler(self, channel, data):
        '''
        Function for handling main lcm channel
        '''
        if (self.ready == True):
            if (self.plant == None):

                # create the plant
                builder = pydrake.systems.framework.DiagramBuilder()
                self.plant, scene_graph = \
                    pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0)
                pydrake.multibody.parsing.Parser(self.plant).AddModelFromFile(
                FindResourceOrThrow(self.modelFile))

                # determing is there is a need to use "weldframes" function
                if (self.weldBody == True):
                    self.plant.WeldFrames(self.plant.world_frame(),
                        self.plant.GetFrameByName("pelvis"), RigidTransform.Identity())
                self.plant.Finalize()
                self.context = self.plant.CreateDefaultContext()

            # TODO (for Michael): bind our more robust decoding mechanisms to python
            self.msg = dairlib.lcmt_robot_output.decode(data)
            self.plant.SetPositions(self.context, self.msg.position)
            self.plant.SetVelocities(self.context, self.msg.velocity)

            # check if the checkboxes are checked
            for name in self.shapes:
                if (self.checkBoxes[name].isChecked() == True):
                    if (self.checkBoxesPrevState[name] == False):
                        self.checkBoxesPrevState[name] = True
                        self.checkBoxChecked(name)
                else:
                    if (self.checkBoxesPrevState[name] == True):
                        self.checkBoxesPrevState[name] = False
                        self.checkBoxNotChecked(name)

                currShape = self.shapes[name]
                next_loc = None

                # determine the next location based on the type of data being
                # currently processed
                if (currShape.category == "kinematic_point"):
                    # Use Drake's CalcPointsPositions to determine where that point is
                    # in the world
                    pt_world = self.plant.CalcPointsPositions(self.context,
                        self.plant.GetFrameByName(currShape.frame),
                        currShape.point, self.plant.world_frame())
                    next_loc = pt_world.transpose()[0]

                elif (currShape.category == "CoM"):
                    next_loc = currShape.point

                self.drawShape(currShape, next_loc)

    def drawShape(self, currShape, next_loc):
        '''
        Function for drawing shapes. Currently this supports lines, points, and
        3D axes
        '''
        # draw a continuous line
        if (currShape.type == "line"):
            # check if there is any previously computed location
            if (len(currShape.prev_loc) == 0):
                currShape.prev_loc = next_loc

            # check if the duration has been initialized
            if (currShape.duration == None or len(currShape.object) == 0):
                currShape.duration = self.msg.utime / 1000000

            # add new line
            d = DebugData()
            d.addLine(currShape.prev_loc, next_loc, radius = currShape.thickness)
            line = vis.showPolyData(d.getPolyData(), "line")

            # set color and transparency of line
            line.setProperty('Color', currShape.color)
            line.setProperty('Alpha', currShape.alpha)

            # add line to the history of current lines drawn
            currShape.object.append(line)

            # visualize and trace line for 'history' seconds
            if ((self.msg.utime / 1000000) - currShape.duration <= currShape.history or currShape.history <= 0):
                # if (self.distance(currShape.prev_loc, next_loc) >= 10e-5):
                    # add new line
                    d = DebugData()
                    d.addLine(currShape.prev_loc, next_loc, radius = currShape.thickness)
                    line = vis.showPolyData(d.getPolyData(), "line")

                    # set color and transparency of line
                    line.setProperty('Color', currShape.color)
                    line.setProperty('Alpha', currShape.alpha)

                    # add line to the history of current lines drawn
                    currShape.object.append(line)
            elif (currShape.history > 0):
                if (len(currShape.object) == 0):
                    currShape.duration = self.msg.utime / 1000000
                else:
                    # if (self.distance(currShape.prev_loc, next_loc) >= 10e-5):
                        # reset the points of the last placed line
                        d = DebugData()
                        d.addLine(currShape.prev_loc, next_loc, radius = currShape.thickness)
                        lastLine = currShape.object.popleft()
                        lastLine.setPolyData(d.getPolyData())
                        currShape.object.append(lastLine)

            currShape.prev_loc = next_loc

        # draw a point
        elif (currShape.type == "point"):
            d = DebugData()
            d.addSphere(next_loc, radius = currShape.radius)
            # create a new point
            if (currShape.object == None):
                currShape.object = vis.showPolyData(d.getPolyData(), "sphere")
                # set color and transparency of point
                currShape.object.setProperty('Color', currShape.color)
                currShape.object.setProperty('Alpha', currShape.alpha)
            else:
                # update the location of the last point
                currShape.object.setPolyData(d.getPolyData())

        # draw a set of axes
        elif (currShape.type == "axis"):
            # get the rotation matrix
            rigTrans = self.plant.EvalBodyPoseInWorld(self.context, self.plant.GetBodyByName(currShape.frame))
            rot_matrix = rigTrans.rotation().matrix().transpose()

            d = DebugData()
            for i in range(3):
                d.addArrow(next_loc, next_loc + (rot_matrix[i]/4), headRadius=0.03, color = currShape.color[i])

            # create the 3 axes
            if (currShape.object == None):
                currShape.object = vis.showPolyData(d.getPolyData(), "axis", colorByName='RGB255')
                currShape.object.setProperty('Alpha', currShape.alpha)
            else:
                # update the location of the last point
                currShape.object.setPolyData(d.getPolyData())

class ObjectToDraw():
    '''
    Wrapper class for any object/shape being drawn
    '''
    def __init__(self, data):
        # set attributes from given data (originating from input JSON file)
        jsonData = eval(str(data))
        self.category = jsonData['category']
        self.name = jsonData['name']

        if (self.category == "kinematic_point"):
            self.frame = jsonData['frame']
            self.point = jsonData['point']

        elif (self.category == "CoM"):
            self.point = [0, 0, 0]

        self.color = jsonData['color']
        self.alpha = jsonData['alpha']
        self.type = jsonData['type']
        self.object = None

        if (self.type == "line"):
            self.thickness = jsonData['thickness']
            self.history = jsonData['history']
            self.prev_loc = []
            self.duration = None
            self.object = deque()

        elif (self.type == "point"):
            self.radius = jsonData['radius']

        elif (self.type == "axis"):
            self.thickness = jsonData['thickness']

    def update(self, otherObject):
        '''
        Function for updating certain attributes of already existing object
        '''
        self.frame = otherObject.frame
        self.point = otherObject.point
        self.color = otherObject.color
        self.alpha = otherObject.alpha

        if (self.type == "line"):
            self.thickness = otherObject.thickness
            self.history = otherObject.history

        elif (self.type == "point"):
            self.radius = otherObject.radius

# Adding a widget but there's nothing in the widget (yet)
panel = VisualizationGui()
panel.show()
panel.setLayout(panel.vbox)
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)
