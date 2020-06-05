import lcm
import threading

from PythonQt.QtGui import *
from PythonQt.QtCore import *

import dairlib.lcmt_robot_output

import director.applogic
import director.mainwindowapp

from pydairlib.common import FindResourceOrThrow

from pydrake.math import RigidTransform
import pydrake.systems.framework
import pydrake.multibody.plant
import pydrake.multibody.parsing
import numpy as np


class TestGui(QWidget):

    def __init__(self, parent = None):

        super(TestGui, self).__init__(parent)
        # Starting a new thread, since we need to block and wait for messages
        handle_thread = threading.Thread(target=self.handle_thread)
        handle_thread.start()

    def handle_thread(self):
        self.channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.lcm = lcm.LCM()
        subscription = self.lcm.subscribe(self.channel, self.state_handler)
        subscription.set_queue_capacity(1)

        # Create the plant (TODO: URDF name a JSON option)
        builder = pydrake.systems.framework.DiagramBuilder()
        self.plant, scene_graph = \
            pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, 0)
        pydrake.multibody.parsing.Parser(self.plant).AddModelFromFile(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"))

        # Fixed-base model (weld-body, or null, a JSON option)
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

        # TODO: pt_body and body_name are JSON options
        # Once you can visualize, play around with these values to see what
        # they do. Look in the .urdf file above to know valid body names
        pt_body = np.array([0,0,0])
        body_name = "toe_left"

        # Use Drake's CalcPointsPositions to determine where that point is
        # in the world
        pt_world = self.plant.CalcPointsPositions(self.context,
            self.plant.GetFrameByName(body_name), pt_body,
            self.plant.world_frame())
        print(pt_world.transpose())
        #TODO instead of printing the position, visualize it

# Adding a widget but there's nothing in the widget (yet)
panel = TestGui()
app.addWidgetToDock(panel, QtCore.Qt.RightDockWidgetArea)

view = applogic.getMainWindow()
