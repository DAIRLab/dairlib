#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
#include "common/discrete_time_filter.h"

namespace dairlib::examples::osc_run {

class FootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FootTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                    drake::systems::Context<double>* context,
                    const std::string& foot_name, const std::string& hip_name,
                    int stance_state);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_clock() const {
    return this->get_input_port(clock_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_target_vel() const {
    return this->get_input_port(target_vel_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_contact_scheduler() const {
    return this->get_input_port(contact_scheduler_port_);
  }

  void SetCommandFilter(double alpha){
    target_vel_filter_->UpdateParameters(alpha);
  }

  void SetFootstepGains(const Eigen::MatrixXd& Kd) { Kd_ = Kd; };

  void SetFootPlacementOffsets(double rest_length, double rest_length_offset, double center_line_offset,
                               double footstep_offset, double mid_foot_height) {
    rest_length_ = rest_length;
    rest_length_offset_ = rest_length_offset;
    lateral_offset_ = center_line_offset;
    sagital_offset_ = footstep_offset;
    mid_foot_height_ = mid_foot_height;
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

  // Foot placement constants
  double rest_length_;
  double rest_length_offset_;
  double lateral_offset_;
  double sagital_offset_;
  double mid_foot_height_;
  double m_;

  // Raibert Footstep Gains
  Eigen::MatrixXd Kd_ = Eigen::MatrixXd::Zero(3, 3);

  bool is_left_foot_;
  int stance_state_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex target_vel_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex clock_port_;
  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex contact_scheduler_port_;

  drake::systems::DiscreteStateIndex initial_foot_pos_idx_;
  drake::systems::DiscreteStateIndex initial_hip_pos_idx_;
  drake::systems::DiscreteStateIndex pelvis_yaw_idx_;
  drake::systems::DiscreteStateIndex pelvis_vel_est_idx_;
  drake::systems::DiscreteStateIndex last_stance_timestamp_idx_;

  std::unique_ptr<FirstOrderLowPassFilter> target_vel_filter_;

};

}  // namespace dairlib::examples::osc_run
