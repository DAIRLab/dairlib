from cassie_utils import makeFixedBaseCassieTreePointer
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger

from pydrake.trajectories import PiecewisePolynomial
import pydrake.multibody.rigid_body_plant as mut


class trajectorySim():

	def __init__(self):
		self.tree = makeFixedBaseCassieTreePointer("Don't know why this string needs to be here")
		self.builder = DiagramBuilder()
		self.plant = self.builder.addSystem(self.tree)

		self.defaultMaterial = mut.CompliantMaterial()
		self.defaultMaterial = self.defaultMaterial.set_dissipation(2)
		self.defaultMaterial = self.defaultMaterial.set_friction(0.7, 0.7)
		self.plant.set_default_compliant_material(self.defaultMaterial)

		self.modelParameters = mut.CompliantContactModelParameters()
		self.modelParameters.characteristic_radius = 2e-4
		self.modelParameters.v_stiction_tolerance = 0.01
		self.plant.set_contact_model_parameters(self.modelParameters)

	#	Declare TrajectorySource??

		self.logger = builder.addSystem(SignalLogger(plant.get_output_port(0).size()));

		builder.Connect(self.plant.state_output_port(), logger.get_input_port());

		self.diagram = builder.Build()
		
		self.simulator = Simulator(self.diagram)
		self.simulator.set_publish_every_time_step(false)
		self.simulator.set_publish_at_initialization(false)
		self.simulator.set_target_realtime_rate(1.0)


	def initializeSimulator(self, time):
		self.simulator.Initialize()
		self.simulator.StepTo(time)

	