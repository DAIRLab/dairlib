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
                    bool visualize_ground = true,
                    const Eigen::Vector3d& normal_W = Eigen::Vector3d(0, 0, 1));

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

/// Given a set of maps constructed from the above functions, construct a
/// vector of state and actuator names in order of their index
std::vector<std::string> createStateAndActuatorNameVectorFromMap(
    const std::map<std::string, int>& pos_map,
    const std::map<std::string, int>& vel_map,
    const std::map<std::string, int>& act_map);

std::vector<std::string> createStateAndActuatorNameVectorFromMapDot(
    const std::map<std::string, int>& pos_map,
    const std::map<std::string, int>& vel_map,
    const std::map<std::string, int>& act_map);

// TODO: The following two functions need to be implemented as a part of
// RBT/Multibody and not as separate functions that take in RBTs. Make the
// change once the codebase shifts to using multibody.

// Given a MultibodyPlant and a state vector, checks if the states are within
// the joint limits
bool JointsWithinLimits(const drake::multibody::MultibodyPlant<double>& plant,
                        Eigen::VectorXd positions, double tolerance = 0.0);

/// Gets the single index of the quaternion position coordinates for a floating
/// base joint. Returns the starting index of the length four quaternion
/// coordinates into the generalized position vector 'q'.
template <typename T>
std::vector<int> QuaternionStartIndices(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Gets the single index of the quaternion position coordinates for a floating
/// base joint. Returns the starting index of the length four quaternion
/// coordinates into the generalized position vector 'q'.
/// If there are no quaternion floating base joints, returns -1.
/// Throws an error if there are multiple quaternion floating base joints.
template <typename T>
int QuaternionStartIndex(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Check whether a MultibodyPlant contains quaternion floating-base joint.
/// Throws an error if there are multiple quaternion floating base joints.
/// TODO: this method should be deprecated
template <typename T>
bool isQuaternion(const drake::multibody::MultibodyPlant<T>& plant);

}  // namespace multibody
}  // namespace dairlib
