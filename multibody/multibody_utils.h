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
    const Eigen::Ref<const drake::VectorX<T>>& state,
    const Eigen::Ref<const drake::VectorX<T>>& input);

/// Update an existing MultibodyPlant context, setting corresponding state and
/// input values.
/// Will only set values that have changed.
template <typename T>
void setContext(const drake::multibody::MultibodyPlant<T>& plant,
                const Eigen::Ref<const drake::VectorX<T>>& state,
                const Eigen::Ref<const drake::VectorX<T>>& input,
                drake::systems::Context<T>* context);

/// Update an existing MultibodyPlant context, setting corresponding positions.
/// Will only set if value if changed from current value.
template <typename T>
void SetPositionsAndVelocitiesIfNew(
    const drake::multibody::MultibodyPlant<T>& plant,
    const Eigen::Ref<const drake::VectorX<T>>& x,
    drake::systems::Context<T>* context);

/// Update an existing MultibodyPlant context, setting corresponding positions.
/// Will only set if value if changed from current value.
template <typename T>
void SetPositionsIfNew(const drake::multibody::MultibodyPlant<T>& plant,
                       const Eigen::Ref<const drake::VectorX<T>>& q,
                       drake::systems::Context<T>* context);

/// Update an existing MultibodyPlant context, setting corresponding velocities.
/// Will only set if value if changed from current value.
template <typename T>
void SetVelocitiesIfNew(const drake::multibody::MultibodyPlant<T>& plant,
                        const Eigen::Ref<const drake::VectorX<T>>& f,
                        drake::systems::Context<T>* context);

/// Update an existing MultibodyPlant context, setting corresponding inputs.
/// Will only set if value if changed from current value.
template <typename T>
void SetInputsIfNew(const drake::multibody::MultibodyPlant<T>& plant,
                    const Eigen::Ref<const drake::VectorX<T>>& u,
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

/// Given a set of maps constructed from the above functions, construct a
/// vector of state and actuator names in order of their index
template <typename T>
std::vector<std::string> createStateNameVectorFromMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a set of maps constructed from the above functions, construct a
/// vector of state and actuator names in order of their index
template <typename T>
std::vector<std::string> createActuatorNameVectorFromMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// \param plant_w_spr
/// \param plant_wo_spr
template <typename T>
Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapPos(
    const drake::multibody::MultibodyPlant<T>& plant_w_spr,
    const drake::multibody::MultibodyPlant<T>& plant_wo_spr);

/// \param plant_w_spr
/// \param plant_wo_spr
template <typename T>
Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapVel(
    const drake::multibody::MultibodyPlant<T>& plant_w_spr,
    const drake::multibody::MultibodyPlant<T>& plant_wo_spr);

// TODO: The following two functions need to be implemented as a part of
// RBT/Multibody and not as separate functions that take in RBTs. Make the
// change once the codebase shifts to using multibody.

// Given a MultibodyPlant and a state vector, checks if the states are within
// the joint limits
bool JointsWithinLimits(const drake::multibody::MultibodyPlant<double>& plant,
                        Eigen::VectorXd positions, double tolerance = 0.0);

/// Gets the single index of the quaternion position coordinates for all
/// floating base joints. Returns the starting index of the length four
/// quaternion coordinates into the generalized position vector 'q'.
template <typename T>
std::vector<int> QuaternionStartIndices(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Gets the single index of the quaternion position coordinates for a floating
/// base joint. Returns the starting index of the length four quaternion
/// coordinates into the generalized position vector 'q'.
/// If there are no quaternion floating base joints, returns -1.
/// Throws an error if there are multiple quaternion floating base joints.
template <typename T>
int QuaternionStartIndex(const drake::multibody::MultibodyPlant<T>& plant);

/// Check whether a MultibodyPlant contains quaternion floating-base joint.
/// Throws an error if there are multiple quaternion floating base joints.
/// TODO: this method should be deprecated
template <typename T>
bool isQuaternion(const drake::multibody::MultibodyPlant<T>& plant);

/// Calculate the 3x3 Inertia tensor of a link about the plant's
/// current center of mass in the specified frame
template <typename T>
std::pair<drake::multibody::RotationalInertia<T>, drake::Vector3<T>> CalcLinkInertiaAboutPlantCom(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::systems::Context<T>& context,
    const drake::multibody::BodyFrame<T>& frame,
    std::string link_name);

/// Modify MultibodyPlant to approximate a single rigid body by
/// assigning most of the plant's mass in the specified link
/// with the specified inertia parameters.
/// Mass ratio sets the maximum link mass/inertia in relation to the total mass/inertia
void MakePlantApproximateRigidBody(
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<double>& plant,
    std::string srbd_body, std::vector<std::string> bodies_to_change,
    Eigen::Vector3d b_R_com,
    drake::multibody::RotationalInertia<double> b_I, double mass, double mass_ratio);

}  // namespace multibody
}  // namespace dairlib
