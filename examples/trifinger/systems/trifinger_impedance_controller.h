#include "common/eigen_utils.h"
#include "dairlib/lcmt_single_finger_impedance_debug.hpp"
#include "dairlib/lcmt_trifinger_impedance_debug.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

struct SingleFingerImpedanceControllerOutputs {
  Eigen::Vector3d commanded_torque = Eigen::Vector3d::Zero();
  Eigen::Vector3d cur_fingertip_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d fingertip_target_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d fingertip_delta_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d fingertip_delta_vel = Eigen::Vector3d::Zero();
};

class TrifingerImpedanceControl : public drake::systems::LeafSystem<double> {
 public:
  TrifingerImpedanceControl(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::string& fingertip_0_name,
      const std::string& fingertip_120_name,
      const std::string& fingertip_240_name,
      const Eigen::Matrix3d& Kp_fingertip_0,
      const Eigen::Matrix3d& Kd_fingertip_0,
      const Eigen::Matrix3d& Kp_fingertip_120,
      const Eigen::Matrix3d& Kd_fingertip_120,
      const Eigen::Matrix3d& Kp_fingertip_240,
      const Eigen::Matrix3d& Kd_fingertip_240);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_fingertips_delta_position() const {
    return this->get_input_port(fingertips_target_port_);
  }
  const drake::systems::OutputPort<double>& get_commanded_torque_port() const {
    return this->get_output_port(commanded_torque_port_);
  }
  const drake::systems::OutputPort<double>& get_impedance_debug_output_port()
      const {
    return this->get_output_port(impedance_debug_output_port_);
  }

 private:
  void UpdateCommandedTorques(const drake::systems::Context<double>& context,
                              TimestampedVector<double>* output) const;

  void CopyTrifingerImpedanceDebugOutput(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_trifinger_impedance_debug* output) const;

  void CopySingleFingerImpedanceDebugOutput(
      const SingleFingerImpedanceControllerOutputs& outputs,
      dairlib::lcmt_single_finger_impedance_debug& output) const;

  void CalcCommandedTorqueForFinger(
      SingleFingerImpedanceControllerOutputs& outputs,
      const Eigen::VectorXd& cur_trifinger_velocities,
      const TimestampedVector<double>* all_fingertips_target,
      const drake::multibody::RigidBody<double>* fingertip_body,
      const drake::multibody::RigidBodyFrame<double>* fingertip_frame,
      const Eigen::MatrixXd& trifinger_mass_matrix, const Eigen::Matrix3d& Kp,
      const Eigen::Matrix3d& Kd, int finger_index) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fingertips_target_port_;
  drake::systems::OutputPortIndex commanded_torque_port_;
  drake::systems::OutputPortIndex impedance_debug_output_port_;

  const std::string& fingertip_0_name_;
  const std::string& fingertip_120_name_;
  const std::string& fingertip_240_name_;

  const Eigen::Matrix3d Kp_fingertip_0_;
  const Eigen::Matrix3d Kd_fingertip_0_;
  const Eigen::Matrix3d Kp_fingertip_120_;
  const Eigen::Matrix3d Kd_fingertip_120_;
  const Eigen::Matrix3d Kp_fingertip_240_;
  const Eigen::Matrix3d Kd_fingertip_240_;

  const drake::multibody::RigidBody<double>* fingertip_0_body_;
  const drake::multibody::RigidBody<double>* fingertip_120_body_;
  const drake::multibody::RigidBody<double>* fingertip_240_body_;
  const drake::multibody::RigidBodyFrame<double>* fingertip_0_frame_;
  const drake::multibody::RigidBodyFrame<double>* fingertip_120_frame_;
  const drake::multibody::RigidBodyFrame<double>* fingertip_240_frame_;
  const drake::multibody::RigidBodyFrame<double>* world_frame_;

  mutable SingleFingerImpedanceControllerOutputs finger_0_outputs_;
  mutable SingleFingerImpedanceControllerOutputs finger_120_outputs_;
  mutable SingleFingerImpedanceControllerOutputs finger_240_outputs_;
};
}  // namespace dairlib::systems
