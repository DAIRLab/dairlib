"""Teleop test to interact with a Drake FEM model."""

import numpy as np
import os.path as op
import sys
from graphviz import Source

from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import Value
from pydrake.geometry import (
    AddContactMaterial,
    DrakeVisualizer,
    GeometryInstance,
    ProximityProperties,
    Sphere,
)
from pydrake.lcm import DrakeLcm, DrakeLcmInterface
from pydrake.math import RigidTransform
from pydrake.multibody.fem import DeformableBodyConfig
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    DiscreteContactApproximation,
)
from pydrake.multibody.tree import PdControllerGains, JointActuatorIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmSubscriberSystem, LcmInterfaceSystem

DAIRLIB_DIR = op.abspath(
    op.dirname(op.dirname(op.dirname(op.dirname(__file__))))
)
sys.path.append(op.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))
import dairlib

# Constants.
kSimpleEEModel = (
    "/mnt/data0/bibit/dairlib/examples/sampling_c3/urdf/"
    + "end_effector_simple_model.urdf"
)
kGroundModel = "/mnt/data0/bibit/dairlib/examples/sampling_c3/urdf/ground.urdf"
X_WORLD_TO_HAND = RigidTransform(
    quaternion=Quaternion(0.5, -0.5, -0.5, -0.5), p=[0, 0.2, 0.2]
)
kYoungsModulus = 3e4
kPoissonsRatio = 0.4
kMassDensity = 1e3
kStiffnessDampingCoefficient = 0.01
kSurfaceFriction = CoulombFriction(1.15, 1.15)
kDissipation = 10.0
TELEOP_SCALE = 0.0002


# Function definitions.
def draw_diagram(diagram, name):
    try:
        dot = diagram.GetGraphvizString()
        out_dot = f"/tmp/{name}.dot"
        with open(out_dot, "w") as f:
            f.write(dot)

        # Turn into png.
        src = Source(dot)
        out_base = f"/mnt/data0/bibit/diagrams/{name}"
        src.render(out_base, format="png", cleanup=True)
        print("Rendered diagram PNG to:", out_base + ".png")
    except Exception as e:
        print("Failed to produce Graphviz representation:", e)


class RadioTeleop3D(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(
            "radio_msg", Value(dairlib.lcmt_radio_out())
        )
        self.DeclareVectorOutputPort("ee_desired_state", 6, self.CalcEEState)

        self.target = np.array([0, 0, 0.1, 0, 0, 0])  # x, y, z, vx, vy, vz

    def CalcEEState(self, context, output):
        radio_msg = self.get_input_port(0).Eval(context)
        if np.abs(radio_msg.channel[0]) > 0.01:
            self.target[0] += TELEOP_SCALE * radio_msg.channel[0]
        if np.abs(radio_msg.channel[1]) > 0.01:
            self.target[1] += TELEOP_SCALE * radio_msg.channel[1]
        if np.abs(radio_msg.channel[2]) > 0.01:
            self.target[2] += TELEOP_SCALE * radio_msg.channel[2]
        ee_desired_state = self.target
        output.set_value(ee_desired_state)


# Start building the plant.
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1.0e-3)

# Step 1:  Add a ground.
parser = Parser(plant, scene_graph)
parser.AddModels(kGroundModel)
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))

# Step 2:  Add the end effector.
ee_index = parser.AddModels(kSimpleEEModel)[0]
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))

# Step 3:  Add a deformable body to the model.
dut = plant.mutable_deformable_model()
deformable_body_config = DeformableBodyConfig()
deformable_body_config.set_youngs_modulus(kYoungsModulus)
deformable_body_config.set_poissons_ratio(kPoissonsRatio)
deformable_body_config.set_mass_density(kMassDensity)
deformable_body_config.set_stiffness_damping_coefficient(
    kStiffnessDampingCoefficient
)
geometry = GeometryInstance(
    X_PG=RigidTransform(p=[0, 0.25, 0.4]), shape=Sphere(0.05), name="sphere"
)
props = ProximityProperties()
AddContactMaterial(kDissipation, None, kSurfaceFriction, props)
geometry.set_proximity_properties(props)
body_id = dut.RegisterDeformableBody(
    geometry_instance=geometry,
    config=deformable_body_config,
    resolution_hint=0.05,
)

# Add PD controllers to the EE actuators so teleop is simple.
Kp = 1e3
Kd = 5.0 * np.sqrt(Kp)
gain = PdControllerGains(p=Kp, d=Kd)
for i in range(plant.num_actuators()):
    plant.get_mutable_joint_actuator(
        JointActuatorIndex(i)
    ).set_controller_gains(gain)

# Step 4:  Finalize the plant.
plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
plant.Finalize()

# Step 5:  Add visualization capabilities:  need to run separate process:
# `python -m pydrake.visualization.meldis`
DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

# Step 6:  Add a radio subscriber.
drake_lcm = DrakeLcm(lcm_url="udpm://239.255.76.67:7667?ttl=0")
lcm = builder.AddSystem(LcmInterfaceSystem(drake_lcm))
radio_sub = builder.AddSystem(
    LcmSubscriberSystem.Make(
        channel="DEFORM_RADIO", lcm_type=dairlib.lcmt_radio_out, lcm=lcm
    )
)
radio_translator = builder.AddSystem(RadioTeleop3D())
builder.Connect(radio_sub.get_output_port(), radio_translator.get_input_port())
builder.Connect(
    radio_translator.get_output_port(),
    plant.get_desired_state_input_port(ee_index),
)

diagram = builder.Build()

# Draw the graphviz representation of the diagram.
draw_diagram(diagram, "fem_teleop")


# Simulate for a bit.
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(1000.0)
