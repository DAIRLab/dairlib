#include "multibody/multibody_utils.h"

#include <set>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace multibody {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::SceneGraph;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::VectorXd;
using std::map;
using std::string;
using std::vector;

bool AreVectorsEqual(const Eigen::Ref<const AutoDiffVecXd>& a,
                     const Eigen::Ref<const AutoDiffVecXd>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (autoDiffToValueMatrix(a) != autoDiffToValueMatrix(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = autoDiffToGradientMatrix(a);
  const Eigen::MatrixXd b_gradient = autoDiffToGradientMatrix(b);
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
VectorX<T> getInput(const MultibodyPlant<T>& plant, const Context<T>& context) {
  if (plant.num_actuators() > 0) {
    VectorX<T> input = plant.EvalEigenVectorInput(
        context, plant.get_actuation_input_port().get_index());
    return input;
  } else {
    return VectorX<T>(0);
  }
}

template <typename T>
std::unique_ptr<Context<T>> createContext(
    const MultibodyPlant<T>& plant, const Eigen::Ref<const VectorX<T>>& state,
    const Eigen::Ref<const VectorX<T>>& input) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context.get(), state);

  plant.get_actuation_input_port().FixValue(context.get(), input);
  return context;
}

template <typename T>
void setContext(const MultibodyPlant<T>& plant,
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
void addFlatTerrain(MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic,
                    Eigen::Vector3d normal_W) {
  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<T> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  const drake::math::RigidTransformd X_WG(
      HalfSpace::MakePose(normal_W, point_W));

  plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                   "collision", friction);

  // Add visual for the ground.
  plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                "visual");
}

/// Construct a map between joint names and position indices
///     <name,index> such that q(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "position[ind]""
///  -Index mapping can also be used as a state mapping (assumes x = [q;v])
template <typename T>
map<string, int> makeNameToPositionsMap(const MultibodyPlant<T>& plant) {
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

  // TODO: once RBT fully deprecated, this block can likely be removed, using
  // default coordinate names from Drake.
  auto floating_bodies = plant.GetFloatingBaseBodies();
  DRAKE_THROW_UNLESS(floating_bodies.size() <= 1);
  for (auto body_index : floating_bodies) {
    const auto& body = plant.get_body(body_index);
    DRAKE_ASSERT(body.has_quaternion_dofs());
    int start = body.floating_positions_start();
    // should be body.name() once RBT is deprecated
    std::string name = "base";
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

/// Construct a map between joint names and velocity indices
///     <name,index> such that v(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "state[ind]"
///  -Index mapping can also be used as a state mapping, AFTER
///     an offset of num_positions is applied (assumes x = [q;v])
template <typename T>
map<string, int> makeNameToVelocitiesMap(const MultibodyPlant<T>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;

  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<T>& joint = plant.get_joint(i);
    // TODO(posa): this "dot" should be removed, it's an anachronism from
    // RBT
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
  // Remove throw once RBT deprecated
  DRAKE_THROW_UNLESS(floating_bodies.size() <= 1);
  for (auto body_index : floating_bodies) {
    const auto& body = plant.get_body(body_index);
    int start = body.floating_velocities_start() - plant.num_positions();
    std::string name = "base";  // should be body.name() once RBT is deprecated
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

  return name_to_index_map;
}

template <typename T>
map<string, int> makeNameToActuatorsMap(const MultibodyPlant<T>& plant) {
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
vector<string> createStateNameVectorFromMap(
    const MultibodyPlant<T>& plant) {
  map<string, int> pos_map = makeNameToPositionsMap(plant);
  map<string, int> vel_map = makeNameToVelocitiesMap(plant);
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
vector<string> createActuatorNameVectorFromMap(
    const MultibodyPlant<T>& plant) {
  map<string, int> act_map = makeNameToActuatorsMap(plant);
  vector<string> actuator_names(act_map.size());

  for (const auto& name_index_pair : act_map) {
    actuator_names[name_index_pair.second] = name_index_pair.first;
  }
  return actuator_names;
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
bool isQuaternion(const MultibodyPlant<T>& plant) {
  return QuaternionStartIndex(plant) != -1;
}

template int QuaternionStartIndex(const MultibodyPlant<double>& plant);  // NOLINT
template int QuaternionStartIndex(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::vector<int> QuaternionStartIndices(const MultibodyPlant<double>& plant);  // NOLINT
template std::vector<int> QuaternionStartIndices(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template bool isQuaternion(const MultibodyPlant<double>& plant);  // NOLINT
template bool isQuaternion(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToPositionsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToPositionsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToVelocitiesMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToVelocitiesMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToActuatorsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToActuatorsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template vector<string> createStateNameVectorFromMap(const MultibodyPlant<double>& plant);  // NOLINT
template vector<string> createStateNameVectorFromMap(const MultibodyPlant<AutoDiffXd>& plant);   // NOLINT
template vector<string> createActuatorNameVectorFromMap(const MultibodyPlant<double>& plant);  // NOLINT
template vector<string> createActuatorNameVectorFromMap(const MultibodyPlant<AutoDiffXd>& plant);   // NOLINT
template void addFlatTerrain<double>(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, double mu_static, double mu_kinetic, Eigen::Vector3d normal_W);   // NOLINT
template VectorX<double> getInput(const MultibodyPlant<double>& plant, const Context<double>& context);  // NOLINT
template VectorX<AutoDiffXd> getInput(const MultibodyPlant<AutoDiffXd>& plant, const Context<AutoDiffXd>& context);  // NOLINT
template std::unique_ptr<Context<double>> createContext(const MultibodyPlant<double>& plant, const Eigen::Ref<const VectorXd>& state, const Eigen::Ref<const VectorXd>& input);  // NOLINT
template std::unique_ptr<Context<AutoDiffXd>> createContext(const MultibodyPlant<AutoDiffXd>& plant, const Eigen::Ref<const AutoDiffVecXd>& state, const Eigen::Ref<const AutoDiffVecXd>& input);  // NOLINT
template void setContext(const MultibodyPlant<double>& plant, const Eigen::Ref<const VectorXd>& state, const Eigen::Ref<const VectorXd>&, Context<double>* context);  // NOLINT
template void setContext(const MultibodyPlant<AutoDiffXd>& plant, const Eigen::Ref<const AutoDiffVecXd>& state, const Eigen::Ref<const AutoDiffVecXd>&, Context<AutoDiffXd>* context);  // NOLINT
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
