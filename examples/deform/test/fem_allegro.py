from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import (
    GeometryInstance,
    ProximityProperties,
    Sphere,
    AddContactMaterial,
)
from pydrake.math import RigidTransform
from pydrake.multibody.fem import DeformableBodyConfig
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    DiscreteContactApproximation,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import DrakeVisualizer

# Constants.
kHandModel = (
    "package://drake_models/allegro_hand_description/urdf/"
    + "allegro_hand_description_right.urdf"
)
X_WORLD_TO_HAND = RigidTransform(
    quaternion=Quaternion(0.5, -0.5, -0.5, -0.5), p=[0, 0.2, 0.2]
)
kYoungsModulus = 3e4
kPoissonsRatio = 0.4
kMassDensity = 1e3
kStiffnessDampingCoefficient = 0.01
kSurfaceFriction = CoulombFriction(1.15, 1.15)
kDissipation = 10.0


# Start building the diagram.
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1.0e-3)


# Step 1:  Add the robot.
def add_robot_hand_to_plant(plant, scene_graph):
    parser = Parser(plant, scene_graph)
    hand_index = parser.AddModelsFromUrl(kHandModel)[0]
    plant.WeldFrames(
        plant.world_frame(), plant.GetFrameByName("hand_root"), X_WORLD_TO_HAND
    )
    return hand_index


hand_index = add_robot_hand_to_plant(plant, scene_graph)


# Step 2:  Add a deformable body to the model.
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

# Turn on SAP and finalize.
plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
plant.Finalize()

# Add visualization capabilities:  need to run separate process:
# `python -m pydrake.visualization.meldis`
DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

diagram = builder.Build()

# Simulate for a bit.
simulator = Simulator(diagram)
simulator.AdvanceTo(10.0)
