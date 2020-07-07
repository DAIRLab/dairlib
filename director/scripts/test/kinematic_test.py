import lcm
import threading, queue

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_robot_output
from director import visualization as vis
from director import lcmUtils
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

        # self.abstract_channel = "OSC_DEBUG"
        # self.abstract_type = "lcmt_osc_output"
        # self.abstract_field = "lcmt_osc_tracking_data"
        # self.abstract_x_index = 3
        # self.abstract_y_index = 3
        # self.abstract_z_index = 3

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

        self.subscription = lcmUtils.addSubscriber(self.channel, messageClass=dairlib.lcmt_robot_output, callback=self.state_handler)

        # abstract_subscription = \
        #     self.lcm.subscribe(self.abstract_channel, self.abstract_handler)
        # abstract_subscription.set_queue_capacity(1)

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


    def readJSONFile(self):
        '''
        Function for reading JSON input file and populating the JSON
        and GUI attributes
        '''

        # load only if input is not empty
        if (self.JSONInput.text != ""):
            self.json_file = self.JSONInput.text
            with open(self.json_file) as json_file:
                self.data = json.load(json_file)

            self.ready = True
            self.modelFile = self.data['model_file']
            if ('weld-body' in self.data):
                self.weldBody = True

            # create each object/shape to be drawn
            for data in self.data['data']:
                newObject = ObjectToDraw(data)
                if (newObject.name not in self.shapes):
                    self.shapes[newObject.name] = newObject
                else:
                    self.shapes[newObject.name].update(newObject)

            # fill the labels for each data with its name and add the reset button
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
                # create appropriate symbol extention
                dotOrLine = " •"
                if (self.shapes[name].type == "point"):
                    dotOrLine = " •"
                else:
                    dotOrLine = " ---"

                # create each checkbox and conditionally add it to the GUI
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
            self.shapes[name].polyline.setProperty('Visible', True)

        elif (self.shapes[name].type == "point"):
            self.shapes[name].sphere.setProperty('Visible', True)

        elif (self.shapes[name].type == "axis"):
            self.shapes[name].axis.setProperty('Visible', True)

    def checkBoxNotChecked(self, name):
        '''
        Function for hiding a shape when its corresponding checkbox is unchecked
        '''
        if (self.shapes[name].type == "line"):
            self.shapes[name].polyline.setProperty('Visible', False)

        elif (self.shapes[name].type == "point"):
            self.shapes[name].sphere.setProperty('Visible', False)

        elif (self.shapes[name].type == "axis"):
            self.shapes[name].axis.setProperty('Visible', False)

    def distance(self, pt1, pt2):
        '''
        Function for computing distance between 2 given 3D points
        '''
        sum = 0
        for i in range(len(pt1)):
            sum += pow(pt2[i] - pt1[i], 2)

        return math.sqrt(sum)


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

    def state_handler(self, msg):
        '''
        Function for handling main lcm channel
        '''
        # start listenning to messages once the JSON file has been read
        if (self.ready == True):
            if (self.plant == None):
                # Create the plant
                builder = pydrake.systems.framework.DiagramBuilder()
                self.plant, scene_graph = \
                    pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0)
                pydrake.multibody.parsing.Parser(self.plant).AddModelFromFile(
                FindResourceOrThrow(self.modelFile))

                # determine if there is a need to use the "weldframes" function
                if (self.weldBody == True):
                    self.plant.WeldFrames(self.plant.world_frame(),
                        self.plant.GetFrameByName("pelvis"), RigidTransform.Identity())
                self.plant.Finalize()
                self.context = self.plant.CreateDefaultContext()

            # TODO (for Michael): bind our more robust decoding mechanisms to python
            self.msg = msg
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

                # define next_loc according to each shape/object to be drawn
                next_loc = None
                if (currShape.category == "kinematic_point"):
                    # Use Drake's CalcPointsPositions to determine where that point is
                    # in the world
                    pt_world = self.plant.CalcPointsPositions(self.context,
                        self.plant.GetFrameByName(currShape.frame),
                        currShape.point, self.plant.world_frame())
                    next_loc = pt_world.transpose()[0]

                elif (currShape.category == "CoM"):
                    print(self.plant.CalcCenterOfMassPosition())
                    next_loc = currShape.point

                self.drawShape(currShape, next_loc)

            # handle flags for clearing line histories
            if (self.clear == True):
                for shape in self.shapes.values():
                    if (shape.type == "line"):
                        om.removeFromObjectModel(shape.polyline)
                        shape.polyline = None
                        shape.points = deque()
                self.clear = False

            # handle flags for deleting objects
            if (self.delete == True):
                for shape in self.shapes.values():
                    if (shape.type == "line"):
                        om.removeFromObjectModel(shape.polyline)

                    elif(shape.type == "point"):
                        om.removeFromObjectModel(shape.sphere)

                    elif(shape.type == "axis"):
                        om.removeFromObjectModel(shape.axis)

                self.delete = False
                self.reset = True

            # handle flags for reseting the GUI
            if (self.reset == True):
                # delete checkboxes from GUI
                for i in reversed(range(self.checkBoxArea.count())):
                    self.checkBoxArea.itemAt(i).widget().deleteLater()

                self.resetBtn.deleteLater()
                self.clearBtn.deleteLater()

                # reset GUI variables
                self.checkBoxes = {}
                self.checkBoxesPrevState = {}
                self.resetBtn = None
                self.clearBtn = None
                self.checkBoxArea = None
                self.reset = False
                self.delete = False
                self.clear = False
                self.ready = False

                # reset JSON attributes
                self.data = None
                self.modelFile = None
                self.weldBody = False
                self.shapes = {}
                self.plant = None

    def drawShape(self, currShape, next_loc):
        '''
        Function for drawing shapes. Currently this supports lines, points, and
        3D axes
        '''
        # draw a continuous line
        if (currShape.type == "line"):
            # check if the duration has been initialized
            if (currShape.duration == None or len(currShape.points) == 0):
                currShape.duration = self.msg.utime / 1000000

            # visualize and trace line for 'history' seconds, adding points at a distance at least 10e-5
            if (((self.msg.utime / 1000000) - currShape.duration <= currShape.history) or currShape.history <= 0):
                if (len(currShape.points) < 2):
                    currShape.points.append(next_loc)
                    d = DebugData()
                    d.addPolyLine(currShape.points, radius=currShape.thickness, color=currShape.color)

                    if (currShape.polyline == None):
                        currShape.polyline = vis.showPolyData(d.getPolyData(), "line")
                        currShape.polyline.setProperty('Color', currShape.color)
                    else:
                        currShape.polyline.setPolyData(d.getPolyData())

                else:
                    if (self.distance(currShape.points[-1], next_loc) >= 10e-5):
                        currShape.points.append(next_loc)
                        d = DebugData()
                        d.addPolyLine(currShape.points, radius=currShape.thickness, color=currShape.color)

                        if (currShape.polyline == None):
                            currShape.polyline = vis.showPolyData(d.getPolyData(), "line")
                        else:
                            currShape.polyline.setPolyData(d.getPolyData())

            elif (currShape.history > 0):
                if (len(currShape.points) == 0):
                    currShape.duration = self.msg.utime / 1000000
                else:
                    # visualize and trace line for 'history' seconds, adding points at a distance at least 10e-5
                    if (self.distance(currShape.points[-1], next_loc) >= 10e-5):
                        currShape.points.popleft()
                        currShape.points.append(next_loc)
                        d = DebugData()
                        d.addPolyLine(currShape.points, radius=currShape.thickness, color=currShape.color)
                        if (currShape.polyline == None):
                            currShape.polyLine = vis.showPolyData(d.getPolyData(), "line")
                        else:
                            currShape.polyline.setPolyData(d.getPolyData())

        # draw a point
        elif (currShape.type == "point"):
            d = DebugData()
            d.addSphere(next_loc, radius = currShape.radius)
            # create a new point
            if (currShape.created == True):
                currShape.sphere = vis.showPolyData(d.getPolyData(), "sphere")
                # set color and transparency of point
                currShape.sphere.setProperty('Color', currShape.color)
                currShape.sphere.setProperty('Alpha', currShape.alpha)
                currShape.created = False
            else:
                # update the location of the last point
                currShape.sphere.setPolyData(d.getPolyData())

        # draw a set of axes
        elif (currShape.type == "axis"):
            # get the rotation matrix
            rigTrans = self.plant.EvalBodyPoseInWorld(self.context, self.plant.GetBodyByName(currShape.frame))
            rot_matrix = rigTrans.rotation().matrix().transpose()

            d = DebugData()
            for i in range(3):
                d.addArrow(next_loc, next_loc + (rot_matrix[i]/4), headRadius=0.03, color = currShape.color[i])

            # create the 3 axes
            if (currShape.created == True):
                currShape.axis = vis.showPolyData(d.getPolyData(), "axis", colorByName='RGB255')
                currShape.axis.setProperty('Alpha', currShape.alpha)
                currShape.created = False
            else:
                # update the location of the last point
                currShape.axis.setPolyData(d.getPolyData())

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
        self.created = True

        if (self.type == "line"):
            self.thickness = jsonData['thickness']
            self.history = jsonData['history']
            self.duration = None
            self.polyline = None
            self.points = deque()

        elif (self.type == "point"):
            self.radius = jsonData['radius']
            self.sphere = None

        elif (self.type == "axis"):
            self.thickness = jsonData['thickness']
            self.axis = None

    def update(self, otherObject):
        '''
        Function for updating certain attributes of already existing object
        '''
        self.color = otherObject.color
        self.alpha = otherObject.alpha

        if (self.category == "kinematic_point"):
            self.frame = otherObject.frame
            self.point = otherObject.point

        elif (self.category == "CoM"):
            self.point = otherObject.point

        if (self.type == "line"):
            self.thickness = otherObject.thickness
            self.history = otherObject.history

        elif (self.type == "point"):
            self.radius = otherObject.radius

        elif (self.type == "axis"):
            self.thickness = otherObject.thickness

# Adding the widget
panel = VisualizationGui()
panel.show()
panel.setLayout(panel.vbox)
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)
