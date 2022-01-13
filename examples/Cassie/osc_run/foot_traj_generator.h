#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

const double REST_LENGTH = 0.8;

namespace dairlib::examples::osc_run {

class FootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FootTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                    drake::systems::Context<double>* context,
                    const std::string& foot_name, const std::string& hip_name,
                    bool relative_feet, std::vector<double> state_durations);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_target_vel_input_port() const {
    return this->get_input_port(target_vel_port_);
  }

  void SetFootstepGains(const Eigen::MatrixXd& Kd) { Kd_ = Kd; };

  void SetFootPlacementOffsets(double center_line_offset,
                               double footstep_offset) {
    center_line_offset_ = center_line_offset;
    footstep_offset_ = footstep_offset;
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  drake::trajectories::PiecewisePolynomial<double> GenerateFlightTraj(
      const drake::systems::Context<double>& context) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::Frame<double>& world_;
  const drake::multibody::Frame<double>& foot_frame_;
  const drake::multibody::Frame<double>& hip_frame_;

  // Foot spline parameters
  std::vector<double> state_durations_;

  // Foot placement constants
  double center_line_offset_;
  double footstep_offset_;

  // Raibert Footstep Gains
  Eigen::MatrixXd Kd_ = Eigen::MatrixXd::Zero(3, 3);

  bool is_left_foot_;
  bool relative_feet_;
  int stance_state_;

  int state_port_;
  int target_vel_port_;
  int fsm_port_;
  int initial_foot_pos_idx_;
  int initial_hip_pos_idx_;
  int pelvis_yaw_idx_;
  int pelvis_vel_est_idx_;
};

}  // namespace dairlib::examples::osc_run
