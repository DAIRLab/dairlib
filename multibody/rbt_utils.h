#include <map>
#include <string>
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace multibody {
namespace utils {

/// Given a RigidBodyTree, builds a map from position name to position index
std::map<std::string, int> makeNameToPositionsMap(
    const RigidBodyTree<double>& tree);

/// Given a RigidBodyTree, builds a map from velocity name to velocity index
std::map<std::string, int> makeNameToVelocitiesMap(
    const RigidBodyTree<double>& tree);

/// Given a RigidBodyTree, builds a map from actuator name to actuator index
std::map<std::string, int> makeNameToActuatorsMap(
    const RigidBodyTree<double>& tree);

}  // namespace utils
}  // namespace multibody
}  // namespace dairlib