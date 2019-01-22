#include "multibody/rbt_utils.h"

namespace dairlib {
namespace multibody {
namespace utils {

using std::cout;
using std::endl;
using std::map;
using std::string;

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
    std::cout << tree.actuators[i].name_ << std::endl;
    name_to_index_map[tree.actuators[i].name_] = i;
  }
  return name_to_index_map;
}

int GetBodyIndexFromName(const RigidBodyTree<double>& tree, std::string name) {
  for (int i = 0; i < tree.get_num_bodies(); i++) {
    if (!tree.get_body(i).get_name().compare(name)) {
      return i;
    }
  }
  // Return -1 if none of the names match
  return -1;
}

bool JointsWithinLimits(const RigidBodyTree<double>& tree, Eigen::VectorXd x,
                        double tolerance) {
  map<string, int> position_map = tree.computePositionNameToIndexMap();
  bool joints_within_limits = true;

  for (auto const& b : tree.get_bodies()) {
    if (!b->has_parent_body()) continue;
    auto const& joint = b->getJoint();
    if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
      auto joint_lim_min_vec = joint.getJointLimitMin();
      auto joint_lim_max_vec = joint.getJointLimitMax();
      const int ind = position_map.at(joint.get_name());

      if (x(ind) < (joint_lim_min_vec(0) + tolerance) ||
          x(ind) > (joint_lim_max_vec(0) - tolerance)) {
        joints_within_limits = false;

        // Debugging output
        cout << "Joint " << joint.get_name() << "with index " << ind
             << " is outside limits." << endl;
        cout << "Min limit: " << joint_lim_min_vec(0) << endl;
        cout << "Max limit: " << joint_lim_max_vec(0) << endl;
      }
    }
  }

  return joints_within_limits;
}


}  // namespace utils
}  // namespace multibody
}  // namespace dairlib
