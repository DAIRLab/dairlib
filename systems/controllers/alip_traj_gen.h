#pragma once

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/filters/s2s_kalman_filter.h"

namespace dairlib {
namespace systems {

/// This class creates predicted center of mass (COM) trajectory of a bipedal
/// robot.
/// The trajectories in horizontal directions (x and y axes) are predicted, and
/// the traj in the vertical direction (z axis) starts/ends at the
/// current/desired height.
/// There is also a port which, along with predicted x and y CoM positions,
/// outputs Lx and Ly predictions using the ALIP model. The state order for
/// this trajectory is [x_com, y_com, Lx, Ly]^T


/// Constructor inputs:
///  @param plant, the MultibodyPlant
///  @param desired_com_height, desired COM height
///  @param unordered_fsm_states, vector of fsm states
///  @param unordered_state_durations, duration of each state in
///         unordered_fsm_states
///  @param contact_points_in_each_state, <position of the points on the bodies,
///         body frame> pairs of plant for calculating the stance foot
///         position (of each state in unordered_fsm_states). If there are two
///         or more pairs, we get the average of the positions.
/// The last three parameters must have the same size.

class ALIPTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  ALIPTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, double desired_com_height,
      const std::vector<int>& unordered_fsm_states,
      const std::vector<double>& unordered_state_durations,
      const std::vector<std::vector<std::pair<
          const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state);

  // Input port getters
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_touchdown_time()
  const {
    return this->get_input_port(touchdown_time_port_);
  }

  // Output port getters
  const drake::systems::OutputPort<double>& get_output_port_alip_state()
  const {
    return this->get_output_port(output_port_alip_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com() const {
    return this->get_output_port(output_port_com_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int GetModeIdx(int fsm_state) const;

  std::pair<Eigen::MatrixXd, Eigen::VectorXd> MakeExponentialTrajAandXi(
      const Eigen::Vector3d& CoM, const Eigen::Vector3d& L,
      const Eigen::Vector3d& stance_foot_pos) const;

  void CalcAlipState(const Eigen::VectorXd& x, int mode_index,
                     const drake::EigenPtr<Eigen::Vector3d>& CoM,
                     const drake::EigenPtr<Eigen::Vector3d>& L,
                     const drake::EigenPtr<Eigen::Vector3d>& stance_pos) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructAlipComTraj(const Eigen::Vector3d& CoM, const Eigen::Vector3d& L,
                    const Eigen::Vector3d& stance_foot_pos, double start_time,
                    double end_time_of_this_fsm_state) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructAlipStateTraj(const Eigen::Vector3d& CoM, const Eigen::Vector3d& L,
                         const Eigen::Vector3d& stance_foot_pos, double start_time,
                         double end_time_of_this_fsm_state) const;

  void CalcComTrajFromCurrent(const drake::systems::Context<double>& context,
                           drake::trajectories::Trajectory<double>* traj) const;

  void CalcAlipTrajFromCurrent(const drake::systems::Context<double>& context,
                               drake::trajectories::Trajectory<double>* traj) const;

  Eigen::MatrixXd CalcA(double com_z) const;

  // Port indices
  int state_port_;
  int fsm_port_;
  int touchdown_time_port_;

  int output_port_alip_state_;
  int output_port_com_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  double desired_com_height_;
  std::vector<int> unordered_fsm_states_;

  std::vector<double> unordered_state_durations_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<std::vector<std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state_;
  const drake::multibody::BodyFrame<double>& world_;

  bool use_com_;
  double m_;
  double foot_spread_lb_ = 0.2;
  double foot_spread_ub_ = 0.5;
  const drake::multibody::Frame<double>& pelvis_frame_;
  const drake::multibody::Frame<double>& toe_left_frame_;
  const drake::multibody::Frame<double>& toe_right_frame_;
  mutable systems::S2SKalmanFilter
      alip_filter_ = systems::S2SKalmanFilter(systems::S2SKalmanFilterData());
  mutable S2SKalmanFilterData filter_data_;
};

}  // namespace systems
}  // namespace dairlib
