#pragma once

#include <map>
#include <string>
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

template <typename T>
drake::VectorX<T> getInput(const drake::multibody::MultibodyPlant<T>& plant,
                           const drake::systems::Context<T>& context);

template <typename T>
std::unique_ptr<drake::systems::Context<T>> createContext(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::VectorX<T>& state, const drake::VectorX<T>& input);

/// Add flat terrain to an initialized, but not finalized, MultibodyPlant
/// and scene graph. Uses the given values for coefficients of friction
template <typename T>
void addFlatTerrain(drake::multibody::MultibodyPlant<T>* plant,
                    drake::geometry::SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic);
template <typename T>
void addTerrain(drake::multibody::MultibodyPlant<T>* plant,
                drake::geometry::SceneGraph<T>* scene_graph,
                double mu_static = 0.8, double mu_kinetic = 0.8,
                Eigen::Vector3d normal_W = Eigen::Vector3d::Zero());

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

// Check whether a MultibodyPlant contains quaternion floating-base joint or not
// WARNING: This function assumes there is only one plant
bool isQuaternion(const drake::multibody::MultibodyPlant<double>& plant);

}  // namespace multibody
}  // namespace dairlib
