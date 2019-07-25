#pragma once

#include <map>
#include <string>
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace multibody {

/// Given a RigidBodyTree, builds a map from position name to position index
std::map<std::string, int> makeNameToPositionsMap(
    const RigidBodyTree<double>& tree);

/// Given a RigidBodyTree, builds a map from velocity name to velocity index
std::map<std::string, int> makeNameToVelocitiesMap(
    const RigidBodyTree<double>& tree);

/// Given a RigidBodyTree, builds a map from actuator name to actuator index
std::map<std::string, int> makeNameToActuatorsMap(
    const RigidBodyTree<double>& tree);

// TODO: The following two functions need to be implemented as a part of
// RBT/Multibody and not as separate functions that take in RBTs. Make the
// change once the codebase shifts to using multibody.

// Given a RigidBodyTree and body name, get the index of the body in the tree
int GetBodyIndexFromName(const RigidBodyTree<double>& tree, std::string name);

// Given a RigidBodyTree and a state vector, checks if the states are within
// the joint limits
bool JointsWithinLimits(const RigidBodyTree<double>& tree, Eigen::VectorXd x,
                        double tolerance = 0.0);

// Check whether a rigid body tree is floating based
bool IsFloatingBase(const RigidBodyTree<double>& tree);

/// `SetZeroQuaternionToIdentity` checks if the quaternion of floating base is 0.
/// If it is, set it to identity quaternion.
/// TODO(yminchen): Shouldn't use this function in the long run. Should fix
/// whatever is causing the 0 quaternion state in the controller.
///
/// Input: the configuration of the robot which must be in floating-base
void SetZeroQuaternionToIdentity(Eigen::VectorXd* q);


}  // namespace multibody
}  // namespace dairlib
