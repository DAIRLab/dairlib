import matplotlib.pyplot as plt

from pydrake.systems.drawing import plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.parsers import PackageMap
from pydrake.multibody.rigid_body_tree import (
    FloatingBaseType,
    RigidBodyTree,
)
from pydrake.multibody.rigid_body_plant import RigidBodyPlant


tree = RigidBodyTree("examples/Cassie/urdf/cassie_v2.urdf")

builder = DiagramBuilder()
robot = builder.AddSystem(RigidBodyPlant(tree))

tree.drawKinematicTree("test.graph")

plot_system_graphviz(diagram)
plt.show()