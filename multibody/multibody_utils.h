#include <map>
#include <string>
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

/// Given a MultiBodyTree, builds a map from position name to position index
std::map<std::string, int> makeNameToPositionsMap(
    const drake::multibody::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from velocity name to velocity index
std::map<std::string, int> makeNameToVelocitiesMap(
    const drake::multibody::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from actuator name to actuator index
std::map<std::string, int> makeNameToActuatorsMap(
    const drake::multibody::MultibodyPlant<double>& plant);

// TODO: The following two functions need to be implemented as a part of
// RBT/Multibody and not as separate functions that take in RBTs. Make the
// change once the codebase shifts to using multibody.


// Given a MultibodyPlant and a state vector, checks if the states are within
// the joint limits
bool JointsWithinLimits(const drake::multibody::MultibodyPlant<double>& plant,
                        Eigen::VectorXd positions, double tolerance = 0.0);

}  // namespace multibody
}  // namespace dairlib
