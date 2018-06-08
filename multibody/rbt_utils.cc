#include "multibody/rbt_utils.h"

namespace dairlib {
namespace multibody {
namespace utils {

map<string, int> makeNameToPositionsMap(const RigidBodyTree<double>& tree) {
  map<string, int> name_to_index_map;

  for (int i = 0; i < tree.get_num_positions(); ++i) {
    name_to_index_map[tree.get_position_name(i)] = i;
  }
  return name_to_index_map;
}

map<string, int> makeNameToVelocitiesMap(const RigidBodyTree<double>& tree) {
  map<string, int> name_to_index_map;

  for (int i = 0; i < tree.get_num_velocities(); ++i) {
    name_to_index_map[tree.get_velocity_name(i)] = i;
  }
  return name_to_index_map;
}
map<string, int> makeNameToActuatorsMap(const RigidBodyTree<double>& tree) {
  map<string, int> name_to_index_map;

  for (int i = 0; i < tree.get_num_actuators(); ++i) {
    name_to_index_map[tree.actuators[i].name_] = i;
  }
  return name_to_index_map;
}

}
}
}
