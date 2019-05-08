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
  VectorX<T> input = plant.EvalEigenVectorInput(context,
        plant.get_actuation_input_port().get_index());
  return input;
}

template <typename T>
std::unique_ptr<Context<T>> createContext(const MultibodyPlant<T>& plant,
    const VectorX<T>& state, const VectorX<T>& input) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context.get(), state);

  // TODO(mposa) Remove > 0 check once fixed upstream in Drake
  if (input.size() > 0) {
    context->FixInputPort(plant.get_actuation_input_port().get_index(), input);
  }
  return context;
}

template <typename T>
void addFlatTerrain(MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
                    double mu_static, double mu_kinetic) {
  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d normal_W(0, 0, 1);
  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<T> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  plant->RegisterCollisionGeometry(
      plant->world_body(), HalfSpace::MakePose(normal_W, point_W),
      HalfSpace(), "collision", friction);

  // Add visual for the ground.
  plant->RegisterVisualGeometry(
      plant->world_body(), HalfSpace::MakePose(normal_W, point_W),
      HalfSpace(), "visual");
}

/// Construct a map between joint names and position indices
///     <name,index> such that q(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "position[ind]""
///  -Index mapping can also be used as a state mapping (assumes x = [q;v])
map<string, int> makeNameToPositionsMap(const MultibodyPlant<double>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;
  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant.get_joint(i);
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

  for (int i = 0; i < plant.num_positions(); ++i) {
    // if index has not already been captured, add it
    if (index_set.find(i) == index_set.end()) {
      name_to_index_map["position[" + std::to_string(i) + "]"] = i;
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
map<string, int> makeNameToVelocitiesMap(const MultibodyPlant<double>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;

  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant.get_joint(i);
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

  for (int i = 0; i < plant.num_velocities(); ++i) {
    // if index has not already been captured, add it
    if (index_set.find(i) == index_set.end()) {
      name_to_index_map["velocity[" + std::to_string(i) + "]"] = i;
    }
  }

  return name_to_index_map;
}

map<string, int> makeNameToActuatorsMap(const MultibodyPlant<double>& plant) {
  map<string, int> name_to_index_map;
  for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
    const drake::multibody::JointActuator<double>& actuator =
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



bool JointsWithinLimits(const drake::multibody::MultibodyPlant<double>& plant,
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

template void addFlatTerrain<double>(MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph, double mu_static, double mu_kinetic);   // NOLINT
template VectorX<double> getInput(const MultibodyPlant<double>& plant, const Context<double>& context);  // NOLINT
template VectorX<AutoDiffXd> getInput(const MultibodyPlant<AutoDiffXd>& plant, const Context<AutoDiffXd>& context);  // NOLINT
template std::unique_ptr<Context<double>> createContext(const MultibodyPlant<double>& plant, const VectorX<double>& state, const VectorX<double>& input);  // NOLINT
template std::unique_ptr<Context<AutoDiffXd>> createContext(const MultibodyPlant<AutoDiffXd>& plant, const VectorX<AutoDiffXd>& state, const VectorX<AutoDiffXd>& input);  // NOLINT
}  // namespace multibody
}  // namespace dairlib
