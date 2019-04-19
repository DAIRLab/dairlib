#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace systems {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;
using systems::OutputVector;

using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteStateIndex;

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(
      const RigidBodyTree<double>&, bool);
  void solveFourbarLinkage(VectorXd q_init,
    double & left_heel_spring,double & right_heel_spring) const;

 private:

  EventStatus Update(const Context<double>& context,
      DiscreteValues<double>* discrete_state) const;

  void CopyStateOut(const Context<double>& context,
      OutputVector<double>* output) const;

  const RigidBodyTree<double>& tree_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> actuatorIndexMap_;

  int left_thigh_ind_ = -1;
  int right_thigh_ind_ = -1;
  int left_heel_spring_ind_ = -1;
  int right_heel_spring_ind_ = -1;

  DiscreteStateIndex state_idx_;
  DiscreteStateIndex time_idx_;

  bool is_floating_base_;
};

}  // namespace systems
}  // namespace dairlib
