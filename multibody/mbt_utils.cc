#include <vector>
#include <set>
#include "multibody/mbt_utils.h"
#include "drake/common/drake_assert.h"

namespace dairlib {
namespace multibody {
namespace utils {

using std::map;
using std::string;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::JointIndex;
using drake::multibody::JointActuatorIndex;


/// Construct a map between joint names and position indices
///     <name,index> such that q(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "position[ind]""
///  -Index mapping can also be used as a state mapping (assumes x = [q;v])
map<string, int> makeNameToPositionsMap(const MultibodyPlant<double>& plant) {
  map<string, int> name_to_index_map;
  std::set<int> index_set;
  for (JointIndex i(0); i < plant.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant.tree().get_joint(i);
    auto name = joint.name();

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector {i};
      auto selectorMatrix = plant.tree().MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(0, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            DRAKE_ABORT();
          }
        }
      }
      if (selector_index == -1) {
        DRAKE_ABORT();
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
    const drake::multibody::Joint<double>& joint = plant.tree().get_joint(i);
    // TODO(posa): this "dot" should be removed, it's an anachronism from
    // RBT
    auto name = joint.name() + "dot";

    if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
      std::vector<JointIndex> index_vector {i};
      auto selectorMatrix = plant.tree().MakeStateSelectorMatrix(index_vector);
      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.cols(); ++j) {
        if (selectorMatrix(1, j) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            DRAKE_ABORT();
          }
        }
      }
      if (selector_index == -1) {
        DRAKE_ABORT();
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
        plant.tree().get_joint_actuator(i);
    auto name = actuator.name();

    if (actuator.joint().num_velocities() == 1 &&
        actuator.joint().num_positions() == 1) {
      std::vector<JointActuatorIndex> index_vector {i};
      auto selectorMatrix =
          plant.tree().MakeActuatorSelectorMatrix(index_vector);

      // find index and add
      int selector_index = -1;
      for (int j = 0; j < selectorMatrix.rows(); ++j) {
        if (selectorMatrix(j, 0) == 1) {
          if (selector_index == -1) {
            selector_index = j;
          } else {
            DRAKE_ABORT();
          }
        }
      }
      if (selector_index == -1) {
        DRAKE_ABORT();
      }

      name_to_index_map[name] = selector_index;
    }
  }
  return name_to_index_map;
}

}  // namespace utils
}  // namespace multibody
}  // namespace dairlib
