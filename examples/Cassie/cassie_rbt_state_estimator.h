#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

#include "attic/multibody/rigidbody_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(const RigidBodyTree<double>&,
                                   Eigen::VectorXd ekf_x_init,
                                   Eigen::VectorXd ekf_b_init,
                                   bool is_floating_base);
  void solveFourbarLinkage(Eigen::VectorXd q_init, double& left_heel_spring,
                           double& right_heel_spring) const;
  Eigen::MatrixXd ExtractRotationMatrix(Eigen::VectorXd ekf_x);
  Eigen::VectorXd ExtractFloatingBaseVelocities(Eigen::VectorXd ekf_x);
  Eigen::VectorXd ExtractFloatingBasePositions(Eigen::VectorXd ekf_x);
  int ComputeNumContacts(Eigen::VectorXd ekf_x);
  Eigen::MatrixXd ExtractContactPositions(Eigen::VectorXd ekf_x);
  Eigen::MatrixXd CreateSkewSymmetricMatrix(Eigen::VectorXd s);
  Eigen::MatrixXd ComputeX(Eigen::VectorXd ekf_x);
  Eigen::MatrixXd ComputeX(Eigen::MatrixXd R, Eigen::VectorXd v,
                           Eigen::VectorXd p, Eigen::MatrixXd d);
  Eigen::MatrixXd ComputeXDot(Eigen::VectorXd ekf_x, Eigen::VectorXd ekf_b,
                              Eigen::VectorXd u);
  Eigen::VectorXd ComputeBiasDot(Eigen::VectorXd ekf_b);

 private:
  void AssignNonFloatingBaseToOutputVector(
      dairlib::systems::OutputVector<double>* output,
      const cassie_out_t& cassie_out) const;

  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyStateOut(const drake::systems::Context<double>& context,
                    dairlib::systems::OutputVector<double>* output) const;

  const RigidBodyTree<double>& tree_;
  Eigen::VectorXd ekf_x_init_;
  Eigen::VectorXd ekf_b_init_;
  bool is_floating_base_;

  const int num_states_total_ = 27;
  const int num_states_required_ = 13;
  const int num_states_bias_ = 6;

  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> actuatorIndexMap_;

  int left_thigh_ind_ = -1;
  int right_thigh_ind_ = -1;
  int left_heel_spring_ind_ = -1;
  int right_heel_spring_ind_ = -1;

  drake::systems::DiscreteStateIndex state_idx_;
  drake::systems::DiscreteStateIndex ekf_x_idx_;
  drake::systems::DiscreteStateIndex ekf_b_idx_;
  drake::systems::DiscreteStateIndex time_idx_;
};

}  // namespace systems
}  // namespace dairlib
