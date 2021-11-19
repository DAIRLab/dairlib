#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc_run {

class FootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, const std::string& hip_name,
      bool isLeftFoot,
      const drake::trajectories::PiecewisePolynomial<double>& foot_traj,
      const drake::trajectories::PiecewisePolynomial<double>& hip_traj,
      bool relative_feet = false, double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_target_vel_input_port() const {
    return this->get_input_port(target_vel_port_);
  }

  void SetFootstepGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd) {
    Kp_ = Kp;
    Kd_ = Kd;
  };

 private:
  drake::trajectories::PiecewisePolynomial<double> GenerateFlightTraj(
      const Eigen::VectorXd& x, double t) const;
  void AddRaibertCorrection(
      const drake::systems::Context<double>& context,
      drake::trajectories::PiecewisePolynomial<double>* traj) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::Frame<double>& world_;
  const drake::multibody::Frame<double>& hip_frame_;

  // Raibert Footstep Gains
  Eigen::MatrixXd Kp_ = Eigen::MatrixXd::Zero(2, 2);
  Eigen::MatrixXd Kd_ = Eigen::MatrixXd::Zero(2, 2);

  drake::trajectories::PiecewisePolynomial<double> foot_traj_;
  drake::trajectories::PiecewisePolynomial<double> hip_traj_;

  bool is_left_foot_;
  bool relative_feet_;
  int state_port_;
  int target_vel_port_;

  int fsm_port_;
};

}  // namespace dairlib::examples::osc_run
