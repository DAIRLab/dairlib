import numpy as np
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree

urdf_path = FindResourceOrThrow(
    "drake/manipulation/models/" +
    "iiwa_description/urdf/iiwa14_primitive_collision.urdf")
tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kFixed)

q = np.array([1, 2, 3, 4, 5, 6, 7])
v = np.array([-1, -2, -3, -4, -5, -6, -7])

cache = tree.doKinematics(q, v)
print(tree.massMatrix(cache))
