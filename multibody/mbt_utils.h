#include <map>
#include <string>
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace dairlib {
namespace multibody {
namespace utils {

/// Given a MultiBodyTree, builds a map from position name to position index
std::map<std::string, int> makeNameToPositionsMap(
    const drake::multibody::multibody_plant::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from velocity name to velocity index
std::map<std::string, int> makeNameToVelocitiesMap(
    const drake::multibody::multibody_plant::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from actuator name to actuator index
std::map<std::string, int> makeNameToActuatorsMap(
    const drake::multibody::multibody_plant::MultibodyPlant<double>& plant);

}  // namespace utils
}  // namespace multibody
}  // namespace dairlib