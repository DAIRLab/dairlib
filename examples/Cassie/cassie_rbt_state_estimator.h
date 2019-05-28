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

#include <unsupported/Eigen/MatrixFunctions>

namespace dairlib {
namespace systems {

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieRbtStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieRbtStateEstimator(
      const RigidBodyTree<double>& tree, Eigen::VectorXd ekf_x_init,
      Eigen::VectorXd ekf_bias_init, bool is_floating_base,
      Eigen::MatrixXd P = Eigen::MatrixXd::Identity(27, 27),
      Eigen::MatrixXd N_prior = 0.01 * Eigen::MatrixXd::Identity(3, 3),
      Eigen::VectorXd gyro_noise_std = 0.1 * Eigen::VectorXd::Ones(3),
      Eigen::VectorXd accel_noise_std = 0.1 * Eigen::VectorXd::Ones(3),
      Eigen::VectorXd contact_noise_std = 0.1 * Eigen::VectorXd::Ones(3),
      Eigen::VectorXd gyro_bias_noise_std = 0.1 * Eigen::VectorXd::Ones(3),
      Eigen::VectorXd accel_bias_noise_std = 0.1 * Eigen::VectorXd::Ones(3),
      Eigen::VectorXd joints_noise_std = 0.1 * Eigen::VectorXd::Ones(16));
  void solveFourbarLinkage(Eigen::VectorXd q_init, double& left_heel_spring,
                           double& right_heel_spring) const;
  Eigen::MatrixXd ExtractRotationMatrix(Eigen::VectorXd ekf_x) const;
  Eigen::VectorXd ExtractFloatingBaseVelocities(Eigen::VectorXd ekf_x) const;
  Eigen::VectorXd ExtractFloatingBasePositions(Eigen::VectorXd ekf_x) const;
  int ComputeNumContacts() const;
  Eigen::MatrixXd ExtractContactPositions(Eigen::VectorXd ekf_x) const;
  Eigen::MatrixXd CreateSkewSymmetricMatrix(Eigen::VectorXd s) const;
  Eigen::MatrixXd ComputeLieExponential(Eigen::VectorXd xi) const;
  Eigen::MatrixXd ComputeTransformationToeLeftWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeTransformationToeRightWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeRotationToeLeftWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeRotationToeRightWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeToeLeftCollisionPointsWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeToeRightCollisionPointsWrtIMU(Eigen::VectorXd q) const;
  Eigen::MatrixXd ComputeToeLeftJacobianWrtIMU(Eigen::VectorXd q,
                                               Eigen::VectorXd p) const;
  Eigen::MatrixXd ComputeToeRightJacobianWrtIMU(Eigen::VectorXd q,
                                                Eigen::VectorXd p) const;
  Eigen::MatrixXd ComputeX(Eigen::VectorXd ekf_x) const;
  Eigen::MatrixXd ComputeX(Eigen::MatrixXd R, Eigen::VectorXd v,
                           Eigen::VectorXd p, Eigen::MatrixXd d) const;
  Eigen::VectorXd ComputeEkfX(Eigen::MatrixXd X) const;
  Eigen::MatrixXd ComputeP(Eigen::VectorXd ekf_p) const;
  Eigen::VectorXd ComputeEkfP(Eigen::MatrixXd P) const;
  Eigen::MatrixXd PredictX(Eigen::VectorXd ekf_x, Eigen::VectorXd ekf_bias,
                           Eigen::VectorXd u, Eigen::VectorXd q,
                           double dt) const;
  Eigen::VectorXd PredictBias(Eigen::VectorXd ekf_bias, double dt) const;
  Eigen::MatrixXd ComputeAdjointOperator(Eigen::VectorXd ekf_x) const;
  Eigen::MatrixXd ComputeA(Eigen::VectorXd ekf_x) const;
  Eigen::MatrixXd ComputeCov(Eigen::VectorXd q) const;
  Eigen::MatrixXd PredictP(Eigen::VectorXd ekf_x, Eigen::VectorXd ekf_p,
                           Eigen::VectorXd q, double dt) const;
  void ComputeUpdateParams(Eigen::VectorXd ekf_x_predicted,
                           Eigen::VectorXd ekf_p_predicted, Eigen::VectorXd q,
                           Eigen::VectorXd& y, Eigen::VectorXd& b,
                           Eigen::MatrixXd& H, Eigen::MatrixXd& N,
                           Eigen::MatrixXd& Pi, Eigen::MatrixXd& X_full,
                           Eigen::MatrixXd& S, Eigen::MatrixXd& K,
                           Eigen::VectorXd& delta) const;
  Eigen::MatrixXd UpdateX(Eigen::MatrixXd X_predicted,
                          Eigen::VectorXd delta) const;
  Eigen::VectorXd UpdateBias(Eigen::VectorXd ekf_bias_predicted,
                             Eigen::VectorXd delta) const;
  Eigen::MatrixXd UpdateP(Eigen::VectorXd ekf_p_predicted, Eigen::MatrixXd H,
                          Eigen::MatrixXd K, Eigen::MatrixXd N) const;

  void set_contacts(std::vector<int> contacts);
  void set_contacts(int left_contact1, int left_contact2, int right_contact1,
                    int right_contact2);

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
  Eigen::VectorXd ekf_bias_init_;
  bool is_floating_base_;
  Eigen::VectorXd gyro_noise_std_;
  Eigen::VectorXd accel_noise_std_;
  Eigen::VectorXd contact_noise_std_;
  Eigen::VectorXd gyro_bias_noise_std_;
  Eigen::VectorXd accel_bias_noise_std_;
  Eigen::VectorXd joints_noise_std_;

  const int num_states_total_ = 27;
  const int num_states_required_ = 13;
  const int num_states_bias_ = 6;
  const int num_inputs_ = 6;
  const int num_contacts_ = 4;
  const int num_joints_ = 16;

  std::vector<int> contacts_;

  Eigen::VectorXd local_collision_pt1_;
  Eigen::VectorXd local_collision_pt2_;
  Eigen::VectorXd local_collision_pt1_hom_;
  Eigen::VectorXd local_collision_pt2_hom_;

  Eigen::VectorXd g_;
  Eigen::MatrixXd N_prior_;

  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> actuatorIndexMap_;

  int left_thigh_ind_ = -1;
  int right_thigh_ind_ = -1;
  int left_heel_spring_ind_ = -1;
  int right_heel_spring_ind_ = -1;

  drake::systems::DiscreteStateIndex state_idx_;
  drake::systems::DiscreteStateIndex ekf_x_idx_;
  drake::systems::DiscreteStateIndex ekf_bias_idx_;
  drake::systems::DiscreteStateIndex ekf_p_idx_;
  drake::systems::DiscreteStateIndex time_idx_;
};

}  // namespace systems
}  // namespace dairlib
