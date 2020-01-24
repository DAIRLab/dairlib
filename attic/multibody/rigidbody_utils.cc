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
  int idx = -1;
  for (int i = 0; i < tree.get_num_bodies(); i++) {
    if (!tree.get_body(i).get_name().compare(name)) {
      idx = i;
    }
  }
  DRAKE_DEMAND(idx >= 0);
  return idx;
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

bool IsFloatingBase(const RigidBodyTree<double>& tree){
  return tree.get_body(1).getJoint().is_floating();
}

void SetZeroQuaternionToIdentity(Eigen::VectorXd* q) {
  if (q->segment(3,4).norm() == 0.0) {
    DRAKE_ASSERT(q->segment(3,4).norm() != 0);
    (*q)(3) = 1.0;
  }
}

}  // namespace multibody
}  // namespace dairlib
