#include <vector>
#include <set>
#include "multibody/multibody_utils.h"
#include "drake/common/drake_assert.h"

namespace dairlib {
namespace multibody {

using std::map;
using std::string;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::geometry::SceneGraph;
using drake::geometry::HalfSpace;
using drake::multibody::JointIndex;
using drake::multibody::JointActuatorIndex;
using Eigen::VectorXd;
using drake::VectorX;
using drake::AutoDiffXd;

template <typename T>
VectorX<T> getInput(const MultibodyPlant<T>& plant, const Context<T>& context) {
  if (plant.num_actuators() > 0) {
    VectorX<T> input = plant.EvalEigenVectorInput(context,
        plant.get_actuation_input_port().get_index());
    return input;
  } else {
    return VectorX<T>(0);
  }
}

template <typename T>
std::unique_ptr<Context<T>> createContext(const MultibodyPlant<T>& plant,
    const VectorX<T>& state, const VectorX<T>& input) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context.get(), state);

  context->FixInputPort(plant.get_actuation_input_port().get_index(), input);

  return context;
}

template <typename T>
void setContext(const MultibodyPlant<T>& plant,
    const VectorX<T>& state, const VectorX<T>& input, Context<T>* context) {
  plant.SetPositionsAndVelocities(context, state);

  context->FixInputPort(plant.get_actuation_input_port().get_index(), input);
}


template <typename T>
void addFlatTerrain(MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
                double mu_static, double mu_kinetic, Eigen::Vector3d normal_W) {
  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<T> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  const drake::math::RigidTransformd X_WG(
      HalfSpace::MakePose(normal_W, point_W));

  plant->RegisterCollisionGeometry(
      plant->world_body(), X_WG, HalfSpace(), "collision", friction);

  // Add visual for the ground.
  plant->RegisterVisualGeometry(
      plant->world_body(), X_WG, HalfSpace(), "visual");
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
      std::vector<JointIndex> index_vector {i};
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
  DRAKE_THROW_UNLESS(floating_bodies.size() <= 1);  //remove once RBT deprecated
  for (auto body_index : floating_bodies) {
    const auto& body = plant.get_body(body_index);
    DRAKE_ASSERT(body.has_quaternion_dofs());
    int start = body.floating_positions_start();
    std::string name = "base";  // should be body.name() once RBT is deprecated
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
      std::vector<JointIndex> index_vector {i};
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
      std::vector<JointActuatorIndex> index_vector {i};
      auto selectorMatrix =
          plant.MakeActuatorSelectorMatrix(index_vector);

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



bool JointsWithinLimits(const MultibodyPlant<double>& plant,
                        VectorXd positions, double tolerance) {
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
bool isQuaternion(const MultibodyPlant<T>& plant) {
  auto unordered_index_set = plant.GetFloatingBaseBodies();
  if (unordered_index_set.empty()) {
    return false;
  }

  auto first_body_idx = unordered_index_set.begin();
  return plant.get_body(*first_body_idx).has_quaternion_dofs();
}



template bool isQuaternion(const MultibodyPlant<double>& plant);  // NOLINT
template bool isQuaternion(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToPositionsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToPositionsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToVelocitiesMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToVelocitiesMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template map<string, int> makeNameToActuatorsMap<double>(const MultibodyPlant<double>& plant);  // NOLINT
template map<string, int> makeNameToActuatorsMap<AutoDiffXd>(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template void addFlatTerrain<double>(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, double mu_static, double mu_kinetic, Eigen::Vector3d normal_W);   // NOLINT
template VectorX<double> getInput(const MultibodyPlant<double>& plant, const Context<double>& context);  // NOLINT
template VectorX<AutoDiffXd> getInput(const MultibodyPlant<AutoDiffXd>& plant, const Context<AutoDiffXd>& context);  // NOLINT
template std::unique_ptr<Context<double>> createContext(const MultibodyPlant<double>& plant, const VectorX<double>& state, const VectorX<double>& input);  // NOLINT
template std::unique_ptr<Context<AutoDiffXd>> createContext(const MultibodyPlant<AutoDiffXd>& plant, const VectorX<AutoDiffXd>& state, const VectorX<AutoDiffXd>& input);  // NOLINT
template void setContext(const MultibodyPlant<double>& plant, const VectorX<double>& state, const VectorX<double>& input, Context<double>* context);  // NOLINT
template void setContext(const MultibodyPlant<AutoDiffXd>& plant, const VectorX<AutoDiffXd>& state, const VectorX<AutoDiffXd>& input, Context<AutoDiffXd>* context);  // NOLINT
}  // namespace multibody
}  // namespace dairlib
