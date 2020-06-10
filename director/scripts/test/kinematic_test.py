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


class TestGui(QWidget):

    def __init__(self, parent = None):

        super(TestGui, self).__init__(parent)
        # Starting a new thread, since we need to block and wait for messages
        handle_thread = threading.Thread(target=self.handle_thread)
        handle_thread.start()

    def handle_thread(self):
        self.channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.lcm = lcm.LCM()
        self.prev_loc = [0, 0, 0]
        self.line = []
        self.prevLine = deque()
        self.json_file = sys.argv[3];
        self.duration = 0
        self.point = None
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
                if (np.all(self.prev_loc == [0, 0, 0])):
                    self.prev_loc = next_loc

                # initialize the duration
                if (self.duration == 0):
                    self.duration = msg.utime / 1000000

                # visualize and trace line for 2 seconds
                if ((msg.utime / 1000000) - self.duration <= jsonData['history']):
                    # add new line
                    d = DebugData()
                    d.addLine(self.prev_loc, next_loc, radius = jsonData['thickness'])
                    line = vis.showPolyData(d.getPolyData(), 'line')

                    # set color and transparency of line
                    line.setProperty('Color', jsonData['color'])
                    line.setProperty('Alpha', jsonData['alpha'])

                    # add line to the history of current lines drawn
                    self.prevLine.append(line);
                else:
                    # reset the points of the last placed line
                    d = DebugData()
                    d.addLine(self.prev_loc, next_loc, radius = jsonData['thickness'])
                    lastLine = self.prevLine.popleft()
                    lastLine.setPolyData(d.getPolyData())
                    self.prevLine.append(lastLine)

                self.prev_loc = next_loc

            # draw a point
            elif (jsonData['type'] == "point"):
                d = DebugData()
                d.addSphere(next_loc, radius = jsonData['radius'])
                # create a new point
                if (self.point == None):
                    self.point = vis.showPolyData(d.getPolyData(), 'sphere')
                    # set color and transparency of point
                    self.point.setProperty('Color', jsonData['color'])
                    self.point.setProperty('Alpha', jsonData['alpha'])
                else:
                    # update the location of the last point
                    self.point.setPolyData(d.getPolyData())

# Adding a widget but there's nothing in the widget (yet)
panel = TestGui()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)

view = applogic.getMainWindow()
