#pragma once

#include <map>
#include <string>
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

template <typename T>
drake::VectorX<T> getInput(const drake::multibody::MultibodyPlant<T>& plant,
                           const drake::systems::Context<T>& context);

/// Create a new MultibodyPlant context and set the corresponding state and
/// input values. Note, this is potentially an expensive operation!
template <typename T>
std::unique_ptr<drake::systems::Context<T>> createContext(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::VectorX<T>& state, const drake::VectorX<T>& input);

/// Update an existing MultibodyPlant context, setiing corresponding state and
/// input values.
template <typename T>
void setContext(const drake::multibody::MultibodyPlant<T>& plant,
    const drake::VectorX<T>& state, const drake::VectorX<T>& input,
    drake::systems::Context<T>* context);

/// Add terrain to an initialized, but not finalized, MultibodyPlant
/// and scene graph. Uses the given values for coefficients of friction.
/// normal_W is the normal direction of the ground (pointing to z as the
/// default direction)
template <typename T>
void addFlatTerrain(drake::multibody::MultibodyPlant<T>* plant,
                    drake::geometry::SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic,
                    Eigen::Vector3d normal_W = Eigen::Vector3d(0, 0, 1));

/// Given a MultiBodyTree, builds a map from position name to position index
template <typename T>
std::map<std::string, int> makeNameToPositionsMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a MultiBodyTree, builds a map from velocity name to velocity index
template <typename T>
std::map<std::string, int> makeNameToVelocitiesMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a MultiBodyTree, builds a map from actuator name to actuator index
template <typename T>
std::map<std::string, int> makeNameToActuatorsMap(
    const drake::multibody::MultibodyPlant<T>& plant);

// TODO: The following two functions need to be implemented as a part of
// RBT/Multibody and not as separate functions that take in RBTs. Make the
// change once the codebase shifts to using multibody.

// Given a MultibodyPlant and a state vector, checks if the states are within
// the joint limits
bool JointsWithinLimits(const drake::multibody::MultibodyPlant<double>& plant,
                        Eigen::VectorXd positions, double tolerance = 0.0);

// Check whether a MultibodyPlant contains quaternion floating-base joint or not
// WARNING: This function assumes there is only one plant
template <typename T>
bool isQuaternion(const drake::multibody::MultibodyPlant<T>& plant);

}  // namespace multibody
}  // namespace dairlib
