from pydrake.geometry import QueryObject
import numpy as np

from pydrake.all import (MultibodyPlant, Parser, DiagramBuilder, AddMultibodyPlantSceneGraph)

# Create a builder for the simulation diagram.
builder = DiagramBuilder()

# Create a MultibodyPlant and Parser
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
parser = Parser(plant, scene_graph)

# Load the URDF file
urdf_path = "examples/sampling_c3/urdf/controller_push_t_white.sdf"
parser.AddModels(urdf_path)

# Finalize the plant
plant.Finalize()

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# Create context
context = plant.GetMyContextFromRoot(diagram_context)

# Get the geometry context
geometry_context = scene_graph.GetMyContextFromRoot(diagram_context)

# Get the QueryObject
query_object = scene_graph.get_query_output_port().Eval(geometry_context)

# Define the query point (example: [x, y, z])
query_point = np.array([-0.01, 0, 0.0])

# Compute signed distances to all geometries
signed_distances = query_object.ComputeSignedDistanceToPoint(query_point)

# Get the SceneGraphInspector to query geometry information
inspector = scene_graph.model_inspector()

# Map geometry id to shape type
geometry_types = {}
for result in signed_distances:
    shape = inspector.GetShape(result.id_G)
    geometry_types[result.id_G] = type(shape).__name__

# Print shape types along with geometry id and signed distance
for result in signed_distances:
    print(f"Geometry ID: {result.id_G}, Signed Distance: {result.distance}, Shape Type: {geometry_types[result.id_G]}")
