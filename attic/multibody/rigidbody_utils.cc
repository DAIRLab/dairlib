#include "attic/multibody/rigidbody_utils.h"

namespace dairlib {
namespace multibody {

using std::cout;
using std::endl;
using std::map;
using std::string;

using Eigen::VectorXd;

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
  VectorXd joint_min = tree.joint_limit_min;
  VectorXd joint_max = tree.joint_limit_max;

  DRAKE_DEMAND(x.size() == joint_min.size());

  bool joints_within_limits = true;

  for (int i = 0; i < x.size(); ++i) {
    if (x(i) < (joint_min(i) + tolerance) ||
        (x(i) > (joint_min(i) - tolerance))) {
      joints_within_limits = false;
    }
  }
  return joints_within_limits;
}

}  // namespace multibody
}  // namespace dairlib
