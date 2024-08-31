import numpy as np
import pdb
from pydrake.all import (MultibodyPlant, Parser, DiagramBuilder, 
                         MultibodyPositionToGeometryPose, Simulator)

# Create a builder for the simulation diagram.
builder = DiagramBuilder()

# Create a MultibodyPlant and Parser
plant = MultibodyPlant(time_step=0.0)
parser = Parser(plant)

# Load the URDF file
urdf_path = "/home/bibit/dairlib/examples/jacktoy/urdf/end_effector_full.urdf"
parser.AddModelFromFile(urdf_path)
pdb.set_trace()

# Finalize the plant
plant.Finalize()

# Add the plant to the diagram
builder.AddSystem(plant)
builder.Build()

# Create context
context = plant.CreateDefaultContext()

# Compute the mass matrix
mass_matrix = plant.CalcMassMatrix(context)

print("Mass matrix:")
print(mass_matrix)

# Compute the center of mass
com = plant.CalcCenterOfMassPositionInWorld(context)
print("Center of mass:")
print(com)

pdb.set_trace()