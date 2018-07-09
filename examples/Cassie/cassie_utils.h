#pragma once

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace dairlib {

/// Construct and create a unique pointer to a RigidBodyTree<double>
/// for the fixed base version of Cassie.
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
std::unique_ptr<RigidBodyTree<double>> makeFixedBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

/// Builds the rigid body tree for a fixed base Cassie
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
void buildFixedBaseCassieTree(RigidBodyTree<double>& tree,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

std::unique_ptr<RigidBodyTree<double>> makeFloatingBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

void buildFloatingBaseCassieTree(RigidBodyTree<double>& tree, 
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

}
