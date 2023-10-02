#pragma once

#include <map>
#include <string>

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

template <typename T>
drake::VectorX<T> GetInput(const drake::multibody::MultibodyPlant<T>& plant,
                           const drake::systems::Context<T>& context);

/// Create a new MultibodyPlant context and set the corresponding state and
/// input values. Note, this is potentially an expensive operation!
template <typename T>
std::unique_ptr<drake::systems::Context<T>> CreateContext(
    const drake::multibody::MultibodyPlant<T>& plant,
    const Eigen::Ref<const drake::VectorX<T>>& state,
    const Eigen::Ref<const drake::VectorX<T>>& input);

/// Update an existing MultibodyPlant context, setting corresponding state and
/// input values.
/// Will only set values that have changed.
template <typename T>
void SetContext(const drake::multibody::MultibodyPlant<T>& plant,
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
// TODO: we might want to add other types of terrain in the future
template <typename T>
void AddFlatTerrain(drake::multibody::MultibodyPlant<T>* plant,
                    drake::geometry::SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic,
                    Eigen::Vector3d normal_W = Eigen::Vector3d(0, 0, 1),
                    bool show_ground = true);

/// Add a box and return a vector of GeometryIds representing the visual and
/// collision geometries
template <typename T>
std::vector<drake::geometry::GeometryId>
AddBox(drake::multibody::MultibodyPlant<T>* plant,
       drake::geometry::SceneGraph<T>* scene_graph,
       const drake::math::RigidTransform<T>& X_WB,
       const Eigen::Vector3d& len_xyz, double mu);


/// Add a series of random boxes
template <typename T>
std::vector<drake::geometry::GeometryId>
AddRandomBoxes(drake::multibody::MultibodyPlant<T>* plant,
               drake::geometry::SceneGraph<T>* scene_graph,
               const Eigen::Vector3d& normal=Eigen::Vector3d::UnitZ());

/// Get the ordered names from a NameTo___Map
std::vector<std::string> ExtractOrderedNamesFromMap(
    const std::map<std::string, int>& map);

/// Given a MultibodyPlant, builds a map from position name to position index
template <typename T>
std::map<std::string, int> MakeNameToPositionsMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a MultiBodyTree, builds a map from velocity name to velocity index
template <typename T>
std::map<std::string, int> MakeNameToVelocitiesMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a MultiBodyTree, builds a map from actuator name to actuator index
template <typename T>
std::map<std::string, int> MakeNameToActuatorsMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a set of maps constructed from the above functions, construct a
/// vector of state and actuator names in order of their index
template <typename T>
std::vector<std::string> CreateStateNameVectorFromMap(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given a set of maps constructed from the above functions, construct a
/// vector of state and actuator names in order of their index
template <typename T>
std::vector<std::string> CreateActuatorNameVectorFromMap(
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
bool HasQuaternion(const drake::multibody::MultibodyPlant<T>& plant);


/// Given a vector expressed in the world frame, re-express that vector in
/// a new frame, F, which has Fz // Wz, and Fx // proj_{Wx-y}(B_{x})
template <typename T>
Eigen::Vector3d ReExpressWorldVector3InBodyYawFrame(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::systems::Context<T>& context,
    const std::string& body_name,
    const Eigen::Vector3d& vec);

template <typename T>
Eigen::Vector3d ReExpressBodyYawVector3InWorldFrame(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::systems::Context<T>& context,
    const std::string& body_name,
    const Eigen::Vector3d& vec);



/// Same as above except only considering the x and y components of 'vec'
template <typename T>
Eigen::Vector2d ReExpressWorldVector2InBodyYawFrame(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::systems::Context<T>& context,
    const std::string& body_name,
    const Eigen::Vector2d& vec);

template <typename T>
Eigen::Matrix3d GetBodyYawRotation_R_WB(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::systems::Context<T>& context,
    const std::string& body_name);


/// Given a map of join position offsets labeled by name, i.e.
/// {'toe_left': 0.02, 'knee_right': .0115}, constructs the vector q_offset,
/// such that the corrected position vector is given by q + q_offset.
/// Any subset of joints can be given, so long as each joint appears at
/// most once. An empty map will return a vector of zeros of length
/// plant.num_positions()
/// @joint_offset_map the map of joint offsets
/// @param plant the plant for which the offsets are to be applied
Eigen::VectorXd MakeJointPositionOffsetFromMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::map<std::string, double>& joint_offset_map);


/// Given a map of join position offsets labeled by name, i.e.
/// {'toe_left': 0.02, 'knee_right': .0115}, constructs the vector q_offset,
/// such that the corrected position vector is given by q + q_offset.
/// Any subset of joints can be given, so long as each joint appears at
/// most once. An empty map will return a vector of zeros of length
/// plant.num_positions()
/// @joint_offset_map the map of joint offsets
/// @param plant the plant for which the offsets are to be applied
Eigen::VectorXd MakeJointPositionOffsetFromMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::map<std::string, double>& joint_offset_map);


/// Computes the matrix for mapping global roll-pitch-yaw angular velocity to
/// quaternion derivatives
/// Ref: equation 16 of https://arxiv.org/pdf/0811.2889.pdf
/// Note: The same calculation exits in Drake's
/// QuaternionFloatingMobilizer<T>::AngularVelocityToQuaternionRateMatrix()
/// This matrix transforms angular velocity wrt the WORLD to d/dt quaternion
Eigen::MatrixXd WToQuatDotMap(const Eigen::Vector4d& q);

// Converts Jacobian wrt qdot to jacobian wrt v
Eigen::MatrixXd JwrtqdotToJwrtv(const Eigen::VectorXd& q,
                                const Eigen::MatrixXd& Jwrtqdot);

}  // namespace multibody
}  // namespace dairlib
