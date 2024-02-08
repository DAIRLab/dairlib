#include "multibody/multibody_utils.h"

#include <set>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace multibody {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::SceneGraph;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::BodyIndex;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::map;
using std::string;
using std::vector;

bool AreVectorsEqual(const Eigen::Ref<const AutoDiffVecXd>& a,
                     const Eigen::Ref<const AutoDiffVecXd>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (ExtractValue(a) != ExtractValue(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = ExtractGradient(a);
  const Eigen::MatrixXd b_gradient = ExtractGradient(b);
  if (a_gradient.rows() != b_gradient.rows() ||
      a_gradient.cols() != b_gradient.cols()) {
    return false;
  }
  return a_gradient == b_gradient;
}

bool AreVectorsEqual(const Eigen::Ref<const VectorXd>& a,
                     const Eigen::Ref<const VectorXd>& b) {
  return a == b;
}

template <typename T>
VectorX<T> GetInput(const MultibodyPlant<T>& plant, const Context<T>& context) {
  if (plant.num_actuators() > 0) {
    VectorX<T> input = plant.get_actuation_input_port().Eval(context);
    return input;
  } else {
    return VectorX<T>(0);
  }
}

template <typename T>
std::unique_ptr<Context<T>> CreateContext(
    const MultibodyPlant<T>& plant, const Eigen::Ref<const VectorX<T>>& state,
    const Eigen::Ref<const VectorX<T>>& input) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context.get(), state);

  plant.get_actuation_input_port().FixValue(context.get(), input);
  return context;
}

template <typename T>
void SetContext(const MultibodyPlant<T>& plant,
                const Eigen::Ref<const VectorX<T>>& state,
                const Eigen::Ref<const VectorX<T>>& input,
                Context<T>* context) {
  SetPositionsIfNew<T>(plant, state.head(plant.num_positions()), context);
  SetVelocitiesIfNew<T>(plant, state.tail(plant.num_velocities()), context);
  SetInputsIfNew<T>(plant, input, context);
}

template <typename T>
void SetPositionsAndVelocitiesIfNew(const MultibodyPlant<T>& plant,
                                    const Eigen::Ref<const VectorX<T>>& x,
                                    Context<T>* context) {
  SetPositionsIfNew<T>(plant, x.head(plant.num_positions()), context);
  SetVelocitiesIfNew<T>(plant, x.tail(plant.num_velocities()), context);
}

template <typename T>
void SetPositionsIfNew(const MultibodyPlant<T>& plant,
                       const Eigen::Ref<const VectorX<T>>& q,
                       Context<T>* context) {
  if (!AreVectorsEqual(q, plant.GetPositions(*context))) {
    plant.SetPositions(context, q);
  }
}

template <typename T>
void SetVelocitiesIfNew(const MultibodyPlant<T>& plant,
                        const Eigen::Ref<const VectorX<T>>& v,
                        Context<T>* context) {
  if (!AreVectorsEqual(v, plant.GetVelocities(*context))) {
    plant.SetVelocities(context, v);
  }
}

template <typename T>
void SetInputsIfNew(const MultibodyPlant<T>& plant,
                    const Eigen::Ref<const VectorX<T>>& u,
                    Context<T>* context) {
  if (!plant.get_actuation_input_port().HasValue(*context) ||
      !AreVectorsEqual(u, plant.get_actuation_input_port().Eval(*context))) {
    plant.get_actuation_input_port().FixValue(context, u);
  }
}

template <typename T>
void AddFlatTerrain(MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic,
                    Eigen::Vector3d normal_W, double stiffness,
                    double dissipation_rate, bool show_ground) {
  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<T> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  const drake::math::RigidTransformd X_WG(
      HalfSpace::MakePose(normal_W, point_W));

  if (stiffness != 0) {
    drake::geometry::ProximityProperties props;
    props.AddProperty("material", "point_contact_stiffness", stiffness);
    props.AddProperty("material", "hunt_crossley_dissipation",
                      dissipation_rate);
    props.AddProperty(drake::geometry::internal::kMaterialGroup,
                      drake::geometry::internal::kFriction, friction);
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                     "collision", props);
  } else {
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                     "collision", friction);
  }

  // Add visual for the ground.
  if (show_ground) {
    plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                  "visual");
  }
}

/// Get the ordered names from a NameTo___Map
vector<string> ExtractOrderedNamesFromMap(const map<string, int>& map,
                                          int index_start) {
  vector<string> names(map.size());
  for (const auto& entry : map) {
    names[entry.second - index_start] = entry.first;
  }
  return names;
}

/// Construct a map between joint names and position indices
///     <name,index> such that q(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "position[ind]""
///  -Index mapping can also be used as a state mapping (assumes x = [q;v])
template <typename T>
map<string, int> MakeNameToPositionsMap(const MultibodyPlant<T>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;
  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<T>& joint = plant.get_joint(i);
    auto name = joint.name();

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector{i};
      auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(0, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            throw std::logic_error("Unable to create selector map.");
          }
        }
      }
      if (selector_index == -1) {
        std::logic_error("Unable to create selector map.");
      }

      name_to_index_map[name] = selector_index;
      index_set.insert(selector_index);
    }
  }

  auto floating_bodies = plant.GetFloatingBaseBodies();
  for (auto body_index : floating_bodies) {
    const auto& body = plant.get_body(body_index);
    DRAKE_ASSERT(body.has_quaternion_dofs());
    int start = body.floating_positions_start();
    std::string name = body.name();
    name_to_index_map[name + "_qw"] = start;
    name_to_index_map[name + "_qx"] = start + 1;
    name_to_index_map[name + "_qy"] = start + 2;
    name_to_index_map[name + "_qz"] = start + 3;
    name_to_index_map[name + "_x"] = start + 4;
    name_to_index_map[name + "_y"] = start + 5;
    name_to_index_map[name + "_z"] = start + 6;
    for (int i = 0; i < 7; i++) {
      index_set.insert(start + i);
    }
  }

  for (int i = 0; i < plant.num_positions(); ++i) {
    // if index has not already been captured, throw an error
    if (index_set.find(i) == index_set.end()) {
      DRAKE_THROW_UNLESS(false);
    }
  }

  return name_to_index_map;
}
/// Construct a map between joint names and position indices
///     <name,index> such that q(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "position[ind]""
///  -Index mapping can also be used as a state mapping (assumes x = [q;v])
template <typename T>
map<string, int> MakeNameToPositionsMap(const MultibodyPlant<T>& plant,
                                        ModelInstanceIndex model_instance) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;
  for (auto i : plant.GetJointIndices(model_instance)) {
    const drake::multibody::Joint<T>& joint = plant.get_joint(i);
    auto name = joint.name();

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector{i};
      auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(0, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            throw std::logic_error("Unable to create selector map.");
          }
        }
      }
      if (selector_index == -1) {
        std::logic_error("Unable to create selector map.");
      }

      name_to_index_map[name] = selector_index;
      index_set.insert(selector_index);
    }
  }

  if (plant.HasUniqueFreeBaseBody(model_instance)) {
    const auto& body = plant.GetUniqueFreeBaseBodyOrThrow(model_instance);
    DRAKE_ASSERT(body.has_quaternion_dofs());
    int start = body.floating_positions_start();
    std::string name = body.name();
    name_to_index_map[name + "_qw"] = start;
    name_to_index_map[name + "_qx"] = start + 1;
    name_to_index_map[name + "_qy"] = start + 2;
    name_to_index_map[name + "_qz"] = start + 3;
    name_to_index_map[name + "_x"] = start + 4;
    name_to_index_map[name + "_y"] = start + 5;
    name_to_index_map[name + "_z"] = start + 6;
    for (int i = 0; i < 7; i++) {
      index_set.insert(start + i);
    }
  }
  // if index has not already been captured, throw an error
  DRAKE_THROW_UNLESS(plant.num_positions(model_instance) == index_set.size());

  return name_to_index_map;
}

/// Construct a map between joint names and velocity indices
///     <name,index> such that v(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "state[ind]"
///  -Index mapping can also be used as a state mapping, AFTER
///     an offset of num_positions is applied (assumes x = [q;v])
template <typename T>
map<string, int> MakeNameToVelocitiesMap(const MultibodyPlant<T>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;

  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<T>& joint = plant.get_joint(i);
    auto name = joint.name() + "dot";

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector{i};
      auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(1, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            throw std::logic_error("Unable to create selector map.");
          }
        }
      }
      if (selector_index == -1) {
        throw std::logic_error("Unable to create selector map.");
      }

      name_to_index_map[name] = selector_index - plant.num_positions();
      index_set.insert(selector_index - plant.num_positions());
    }
  }

  auto floating_bodies = plant.GetFloatingBaseBodies();
  for (auto body_index : floating_bodies) {
    const auto& body = plant.get_body(body_index);
    int start = body.floating_velocities_start() - plant.num_positions();
    std::string name = body.name();
    name_to_index_map[name + "_wx"] = start;
    name_to_index_map[name + "_wy"] = start + 1;
    name_to_index_map[name + "_wz"] = start + 2;
    name_to_index_map[name + "_vx"] = start + 3;
    name_to_index_map[name + "_vy"] = start + 4;
    name_to_index_map[name + "_vz"] = start + 5;
    for (int i = 0; i < 6; i++) {
      index_set.insert(start + i);
    }
  }

  for (int i = 0; i < plant.num_velocities(); ++i) {
    // if index has not already been captured, throw an error
    if (index_set.find(i) == index_set.end()) {
      DRAKE_THROW_UNLESS(false);
    }
  }

  // if index has not already been captured, throw an error
  DRAKE_THROW_UNLESS(plant.num_velocities() == index_set.size());
  return name_to_index_map;
}

/// Construct a map between joint names and velocity indices
///     <name,index> such that v(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "state[ind]"
///  -Index mapping can also be used as a state mapping, AFTER
///     an offset of num_positions is applied (assumes x = [q;v])
template <typename T>
map<string, int> MakeNameToVelocitiesMap(const MultibodyPlant<T>& plant,
                                         ModelInstanceIndex model_instance) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;

  for (auto i : plant.GetJointIndices(model_instance)) {
    const drake::multibody::Joint<T>& joint = plant.get_joint(i);
    auto name = joint.name() + "dot";

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector{i};
      auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(1, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            throw std::logic_error("Unable to create selector map.");
          }
        }
      }
      if (selector_index == -1) {
        throw std::logic_error("Unable to create selector map.");
      }

      name_to_index_map[name] = selector_index - plant.num_positions();
      index_set.insert(selector_index - plant.num_positions());
    }
  }

  if (plant.HasUniqueFreeBaseBody(model_instance)) {
    const auto& body = plant.GetUniqueFreeBaseBodyOrThrow(model_instance);
    int start = body.floating_velocities_start() - plant.num_positions();
    std::string name = body.name();
    name_to_index_map[name + "_wx"] = start;
    name_to_index_map[name + "_wy"] = start + 1;
    name_to_index_map[name + "_wz"] = start + 2;
    name_to_index_map[name + "_vx"] = start + 3;
    name_to_index_map[name + "_vy"] = start + 4;
    name_to_index_map[name + "_vz"] = start + 5;
    for (int i = 0; i < 6; i++) {
      index_set.insert(start + i);
    }
  }

  // if index has not already been captured, throw an error
  DRAKE_THROW_UNLESS(plant.num_velocities(model_instance) == index_set.size());
  return name_to_index_map;
}

template <typename T>
map<string, int> MakeNameToActuatorsMap(const MultibodyPlant<T>& plant) {
  map<string, int> name_to_index_map;
  for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
    const drake::multibody::JointActuator<T>& actuator =
        plant.get_joint_actuator(i);
    auto name = actuator.name();

    if (actuator.joint().num_velocities() == 1 &&
        actuator.joint().num_positions() == 1) {
      std::vector<JointActuatorIndex> index_vector{i};
      auto selectorMatrix = plant.MakeActuatorSelectorMatrix(index_vector);

      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.rows(); ++j) {
        if (selectorMatrix(j, 0) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            throw std::logic_error("Unable to create selector map.");
          }
        }
      }
      if (selector_index == -1) {
        throw std::logic_error("Unable to create selector map.");
      }

      name_to_index_map[name] = selector_index;
    }
  }
  return name_to_index_map;
}

template <typename T>
vector<string> CreateStateNameVectorFromMap(const MultibodyPlant<T>& plant) {
  map<string, int> pos_map = MakeNameToPositionsMap(plant);
  map<string, int> vel_map = MakeNameToVelocitiesMap(plant);
  vector<string> state_names(pos_map.size() + vel_map.size());

  for (const auto& name_index_pair : pos_map) {
    state_names[name_index_pair.second] = name_index_pair.first;
  }
  for (const auto& name_index_pair : vel_map) {
    state_names[name_index_pair.second + pos_map.size()] =
        name_index_pair.first;
  }
  return state_names;
}

template <typename T>
vector<string> CreateActuatorNameVectorFromMap(const MultibodyPlant<T>& plant) {
  map<string, int> act_map = MakeNameToActuatorsMap(plant);
  return ExtractOrderedNamesFromMap(act_map);
}

template <typename T>
Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapPos(
    const drake::multibody::MultibodyPlant<T>& plant_w_spr,
    const drake::multibody::MultibodyPlant<T>& plant_wo_spr) {
  const std::map<string, int>& pos_map_w_spr =
      multibody::MakeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::MakeNameToPositionsMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(plant_wo_spr.num_positions(),
                                              plant_w_spr.num_positions());
  for (auto pos_pair_wo_spr : pos_map_wo_spr) {
    bool successfully_added = false;
    for (auto pos_pair_w_spr : pos_map_w_spr) {
      if (pos_pair_wo_spr.first == pos_pair_w_spr.first) {
        ret(pos_pair_wo_spr.second, pos_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  return ret;
}

template <typename T>
Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapVel(
    const drake::multibody::MultibodyPlant<T>& plant_w_spr,
    const drake::multibody::MultibodyPlant<T>& plant_wo_spr) {
  const std::map<string, int>& vel_map_w_spr =
      multibody::MakeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::MakeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(plant_wo_spr.num_velocities(),
                                              plant_w_spr.num_velocities());
  for (auto vel_pair_wo_spr : vel_map_wo_spr) {
    bool successfully_added = false;
    for (auto vel_pair_w_spr : vel_map_w_spr) {
      if (vel_pair_wo_spr.first == vel_pair_w_spr.first) {
        ret(vel_pair_wo_spr.second, vel_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  return ret;
}

bool JointsWithinLimits(const MultibodyPlant<double>& plant, VectorXd positions,
                        double tolerance) {
  VectorXd joint_min = plant.GetPositionLowerLimits();
  VectorXd joint_max = plant.GetPositionUpperLimits();

  bool joints_within_limits = true;

  for (int i = 0; i < positions.size(); ++i) {
    if (positions(i) < (joint_min(i) + tolerance) ||
        (positions(i) > (joint_min(i) - tolerance))) {
      joints_within_limits = false;
    }
  }
  return joints_within_limits;
}

template <typename T>
std::vector<int> QuaternionStartIndices(const MultibodyPlant<T>& plant) {
  std::vector<int> quat_start;
  auto bodies = plant.GetFloatingBaseBodies();
  for (auto body : bodies) {
    if (plant.get_body(body).has_quaternion_dofs()) {
      quat_start.push_back(plant.get_body(body).floating_positions_start());
    }
  }
  return quat_start;
}

template <typename T>
int QuaternionStartIndex(const MultibodyPlant<T>& plant) {
  std::vector<int> quat_start = QuaternionStartIndices(plant);
  if (quat_start.size() > 1) {
    throw std::runtime_error(
        "Called QuaternionStartIndex(plant) with "
        "multiple quaternion floating bases.");
  } else if (quat_start.size() == 0) {
    return -1;
  } else {
    return quat_start.at(0);
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
bool HasQuaternion(const MultibodyPlant<T>& plant) {
  return QuaternionStartIndex(plant) != -1;
}

template <typename T>
Vector3d ReExpressWorldVector3InBodyYawFrame(const MultibodyPlant<T>& plant,
                                             const Context<T>& context,
                                             const std::string& body_name,
                                             const Vector3d& vec) {
  Vector3d pelvis_x =
      plant.GetBodyByName(body_name).EvalPoseInWorld(context).rotation().col(0);
  double yaw = atan2(pelvis_x(1), pelvis_x(0));
  return Vector3d(cos(yaw) * vec(0) + sin(yaw) * vec(1),
                  -sin(yaw) * vec(0) + cos(yaw) * vec(1), vec(2));
}

template <typename T>
Vector2d ReExpressWorldVector2InBodyYawFrame(const MultibodyPlant<T>& plant,
                                             const Context<T>& context,
                                             const std::string& body_name,
                                             const Vector2d& vec) {
  Vector3d pelvis_x =
      plant.GetBodyByName(body_name).EvalPoseInWorld(context).rotation().col(0);
  double yaw = atan2(pelvis_x(1), pelvis_x(0));
  return Vector2d(cos(yaw) * vec(0) + sin(yaw) * vec(1),
                  -sin(yaw) * vec(0) + cos(yaw) * vec(1));
}

VectorXd MakeJointPositionOffsetFromMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::map<std::string, double>& joint_offset_map) {
  VectorXd q_offset = VectorXd::Zero(plant.num_positions());
  const auto pos_map = multibody::MakeNameToPositionsMap(plant);
  for (const auto& [joint_name, joint_offset] : joint_offset_map) {
    DRAKE_DEMAND(pos_map.count(joint_name) > 0);
    q_offset(pos_map.at(joint_name)) = joint_offset;
  }
  return q_offset;
}

Eigen::MatrixXd WToQuatDotMap(const Eigen::Vector4d& q) {
  // clang-format off
  Eigen::MatrixXd ret(4,3);
  ret <<  -q(1), -q(2), -q(3),
      q(0),  q(3), -q(2),
      -q(3),  q(0),  q(1),
      q(2), -q(1),  q(0);
  ret *= 0.5;
  // clang-format on
  return ret;
}

Eigen::MatrixXd JwrtqdotToJwrtv(const Eigen::VectorXd& q,
                                const Eigen::MatrixXd& Jwrtqdot) {
  //[J_1:4, J_5:end] * [WToQuatDotMap, 0] = [J_1:4 * WToQuatDotMap, J_5:end]
  //                   [      0      , I]
  DRAKE_DEMAND(Jwrtqdot.cols() == q.size());

  Eigen::MatrixXd ret(Jwrtqdot.rows(), q.size() - 1);
  ret << Jwrtqdot.leftCols<4>() * WToQuatDotMap(q.head<4>()),
      Jwrtqdot.rightCols(q.size() - 4);
  return ret;
}

template int QuaternionStartIndex(const MultibodyPlant<double>& plant);  // NOLINT
template int QuaternionStartIndex(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::vector<int> QuaternionStartIndices(const MultibodyPlant<double>& plant);  // NOLINT
template std::vector<int> QuaternionStartIndices(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template bool HasQuaternion(const MultibodyPlant<double>& plant);  // NOLINT
template bool HasQuaternion(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template Vector3d ReExpressWorldVector3InBodyYawFrame(const MultibodyPlant<double>& plant, const Context<double>& context, const std::string& body_name, const Vector3d& vec); //NOLINT
template Vector2d ReExpressWorldVector2InBodyYawFrame(const MultibodyPlant<double>& plant, const Context<double>& context, const std::string& body_name, const Vector2d& vec); //NOLINT
template map<string, int> MakeNameToPositionsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> MakeNameToPositionsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd> &plant);  // NOLINT
template map<string, int> MakeNameToPositionsMap<double>(const MultibodyPlant<double>& plant, drake::multibody::ModelInstanceIndex);  // NOLINT
template map<string, int> MakeNameToPositionsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant, drake::multibody::ModelInstanceIndex);  // NOLINT
template map<string, int> MakeNameToVelocitiesMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> MakeNameToVelocitiesMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> MakeNameToVelocitiesMap<double>(const MultibodyPlant<double>& plant, drake::multibody::ModelInstanceIndex);  // NOLINT
template map<string, int> MakeNameToVelocitiesMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant, drake::multibody::ModelInstanceIndex);  // NOLINT
template map<string, int> MakeNameToActuatorsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> MakeNameToActuatorsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template vector<string> CreateStateNameVectorFromMap(const MultibodyPlant<double>& plant);  // NOLINT
template vector<string> CreateStateNameVectorFromMap(const MultibodyPlant<AutoDiffXd>& plant);   // NOLINT
template vector<string> CreateActuatorNameVectorFromMap(const MultibodyPlant<double>& plant);  // NOLINT
template vector<string> CreateActuatorNameVectorFromMap(const MultibodyPlant<AutoDiffXd>& plant);   // NOLINT
template Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapPos(const drake::multibody::MultibodyPlant<double>& plant_w_spr, const drake::multibody::MultibodyPlant<double>& plant_wo_spr);   // NOLINT
template Eigen::MatrixXd CreateWithSpringsToWithoutSpringsMapVel(const drake::multibody::MultibodyPlant<double>& plant_w_spr, const drake::multibody::MultibodyPlant<double>& plant_wo_spr);   // NOLINT
template void AddFlatTerrain<double>(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, double mu_static, double mu_kinetic, Eigen::Vector3d normal_W, double stiffness, double dissipation_rate, bool show_ground);  // NOLINT
template VectorX<double> GetInput(const MultibodyPlant<double>& plant, const Context<double>& context);  // NOLINT
template VectorX<AutoDiffXd> GetInput(const MultibodyPlant<AutoDiffXd>& plant, const Context<AutoDiffXd>& context);  // NOLINT
template std::unique_ptr<Context<double>> CreateContext(const MultibodyPlant<double>& plant, const Eigen::Ref<const VectorXd>& state, const Eigen::Ref<const VectorXd>& input);  // NOLINT
template std::unique_ptr<Context<AutoDiffXd>> CreateContext(const MultibodyPlant<AutoDiffXd>& plant, const Eigen::Ref<const AutoDiffVecXd>& state, const Eigen::Ref<const AutoDiffVecXd>& input);  // NOLINT
template void SetContext(const MultibodyPlant<double>& plant, const Eigen::Ref<const VectorXd>& state, const Eigen::Ref<const VectorXd>&, Context<double>* context);  // NOLINT
template void SetContext(const MultibodyPlant<AutoDiffXd>& plant, const Eigen::Ref<const AutoDiffVecXd>& state, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>* context);  // NOLINT
template void SetPositionsAndVelocitiesIfNew(const MultibodyPlant<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>*);  // NOLINT
template void SetPositionsAndVelocitiesIfNew(const MultibodyPlant<double>&, const Eigen::Ref<const VectorXd>&, Context<double>*);  // NOLINT
template void SetPositionsIfNew(const MultibodyPlant<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>*);  // NOLINT
template void SetPositionsIfNew(const MultibodyPlant<double>&, const Eigen::Ref<const VectorXd>&, Context<double>*);  // NOLINT
template void SetVelocitiesIfNew(const MultibodyPlant<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>*);  // NOLINT
template void SetVelocitiesIfNew(const MultibodyPlant<double>&, const Eigen::Ref<const VectorXd>&, Context<double>*);  // NOLINT
template void SetInputsIfNew(const MultibodyPlant<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>*);  // NOLINT
template void SetInputsIfNew(const MultibodyPlant<double>&, const Eigen::Ref<const VectorXd>&, Context<double>*);  // NOLINT
}  // namespace multibody
}  // namespace dairlib
