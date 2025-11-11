"""Teleop test to interact with a Drake FEM model."""

import numpy as np
import os.path as op
import select
import sys
from graphviz import Source

from pydrake.common.value import Value
from pydrake.geometry import (
    AddContactMaterial,
    DrakeVisualizer,
    GeometryInstance,
    ProximityProperties,
    Sphere,
)
from pydrake.lcm import DrakeLcm
from pydrake.math import RigidTransform
from pydrake.multibody.fem import DeformableBodyConfig
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    DiscreteContactApproximation,
)
from pydrake.multibody.tree import (
    PdControllerGains,
    JointActuatorIndex,
    LinearSpringDamper,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmSubscriberSystem, LcmInterfaceSystem

DAIRLIB_DIR = op.abspath(
    op.dirname(op.dirname(op.dirname(op.dirname(__file__))))
)
sys.path.append(op.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))
import dairlib

# Constants.
EXAMPLE_DIR = "/mnt/data0/bibit/dairlib/examples/deform"
POINT_MODEL = op.join(EXAMPLE_DIR, "models", "small_xyz_point.urdf")
kSimpleEEModel = op.join(EXAMPLE_DIR, "models", "ee_flat.urdf")
# kSimpleEEModel = (
#     "/mnt/data0/bibit/dairlib/examples/sampling_c3/urdf/"
#     + "end_effector_simple_model.urdf"
# )
kEEInitPos = np.array([0.25, 0.0, 0.25])
kObjInitPos = np.array([0.0, 0.25, 0.4])
kGroundModel = op.join(EXAMPLE_DIR, "..", "sampling_c3", "urdf", "ground.urdf")
kYoungsModulus = 3e4
kPoissonsRatio = 0.4
kMassDensity = 1e3
kStiffnessDampingCoefficient = 0.01
kSurfaceFriction = CoulombFriction(1.15, 1.15)
kDissipation = 10.0
TELEOP_SCALE = 0.0002
RADIUS = 0.05
VERTEX_POSITIONS = RADIUS * np.array(
    [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1],
        [0, 0, -1],
    ]
)
VERTEX_POSITIONS[:, 2] += RADIUS * 1.5
VERTEX_CONNECTIONS = [
    (5, 1),
    (5, 2),
    (5, 3),
    (5, 4),
    (6, 1),
    (6, 2),
    (6, 3),
    (6, 4),
    (1, 2),
    (2, 3),
    (3, 4),
    (4, 1),
    (0, 1),
    (0, 2),
    (0, 3),
    (0, 4),
    (0, 5),
    (0, 6),
]
SPRING_STIFFNESS = 1e2
DAMPING_COEFFICIENT = 1e0


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
        self.target[:3] = kEEInitPos

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
parser.SetAutoRenaming(True)
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
    X_PG=RigidTransform(p=kObjInitPos), shape=Sphere(RADIUS), name="sphere"
)
props = ProximityProperties()
AddContactMaterial(kDissipation, None, kSurfaceFriction, props)
geometry.set_proximity_properties(props)
body_id = dut.RegisterDeformableBody(
    geometry_instance=geometry,
    config=deformable_body_config,
    resolution_hint=1.0,  # really large means it will become octohedron
)

# Step 4:  Add a simplified spring-damper model for comparing with FEM model.
body_idxs = []
for i, _ in enumerate(VERTEX_POSITIONS):
    model_idx = parser.AddModels(POINT_MODEL)[0]
    body_idxs.append(model_idx)
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", model_idx),
        RigidTransform(),
    )
for i, j in VERTEX_CONNECTIONS:
    free_length = np.linalg.norm(VERTEX_POSITIONS[i] - VERTEX_POSITIONS[j])
    print(free_length)
    body_i = plant.GetBodyByName("pt", body_idxs[i])
    body_j = plant.GetBodyByName("pt", body_idxs[j])
    plant.AddForceElement(
        LinearSpringDamper(
            bodyA=body_i,
            p_AP=np.zeros(3),
            bodyB=body_j,
            p_BQ=np.zeros(3),
            free_length=free_length.item(),
            stiffness=SPRING_STIFFNESS,
            damping=DAMPING_COEFFICIENT,
        )
    )

# Step 5:  Add PD controllers to the EE actuators so teleop is simple.
Kp = 1e3
Kd = 5.0 * np.sqrt(Kp)
gain = PdControllerGains(p=Kp, d=Kd)
for i in range(plant.num_actuators()):
    plant.get_mutable_joint_actuator(
        JointActuatorIndex(i)
    ).set_controller_gains(gain)

# Step 6:  Finalize the plant.
plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
plant.Finalize()
plant.SetDefaultPositions(ee_index, kEEInitPos)
for i, pos in enumerate(VERTEX_POSITIONS):
    plant.SetDefaultPositions(body_idxs[i], pos)

# Step 7:  Add visualization capabilities:  need to run separate process:
# `python -m pydrake.visualization.meldis`
DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

# Step 8:  Add a radio subscriber.
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


# Start interactive simulation loop.
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
try:
    print("Running simulation: press Enter to pause and q+Enter to quit.")
    # Simulate while allowing breakpoint pauses.
    while True:
        # Run simulation until keypress.
        while True:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                if line.strip().lower() == "q":
                    print("Quitting simulation loop.")
                    raise SystemExit
                break

            # Advance by a small timestep to keep simulation responsive.
            ctx = simulator.get_mutable_context()
            next_t = ctx.get_time() + 0.05
            simulator.AdvanceTo(next_t)

        # Paused:  enter interactive breakpoint for inspection.
        ctx = simulator.get_mutable_context()
        plant_ctx = diagram.GetMutableSubsystemContext(plant, ctx)
        print(f"Paused at t={ctx.get_time():.6f}.")
        breakpoint()
except (KeyboardInterrupt, SystemExit):
    print("Simulation terminated by user.")

# simulator.AdvanceTo(1000.0)
# for i in range(20):
#     simulator.AdvanceTo(1.0)
#     breakpoint()
#     plant_context = diagram.GetMutableSubsystemContext(
#         plant, simulator.get_mutable_context()
#     )
#     ### Properties of the DeformableModel.
#     # Positions are of shape (3, n_vertices).  All expressed in world frame.
#     positions = plant.deformable_model().GetPositions(plant_context, body_id)
#     # Shape (3, 2*n_vertices) with positions in [:, :n_vertices] and velocities
#     # in [:, n_vertices:]
#     ps_and_vs = plant.deformable_model().GetPositionsAndVelocities(
#         plant_context, body_id
#     )
#     # Shape (3*n_vertices,), order [v1x, v1y, v1z, v2x, v2y, v2z, ...]
#     ref_pos = plant.deformable_model().GetReferencePositions(body_id)
#     n_bodies = plant.deformable_model().num_bodies()  # should be 1

#     ### Properties of the DeformableBody.
#     # Unsure what these things are but maybe they'd be useful.
#     is_ephemeral = plant.deformable_model().GetBody(body_id).is_ephemeral()
#     model_instance = plant.deformable_model().GetBody(body_id).model_instance()
#     n_dofs = plant.deformable_model().GetBody(body_id).num_dofs()
#     ref_pos = plant.deformable_model().GetBody(body_id).reference_positions()

#     ### Looking at the FEM model.
#     # This is not python-binded:
#     # fem_model = plant.deformable_model().GetFemModel(body_id)
