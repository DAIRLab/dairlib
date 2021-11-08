#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "multibody/multibody_utils.h"
#include "multibody/single_rigid_body_plant.h"
#include "systems/framework/output_vector.h"

/// System to generate trajectories and foot targets to warmstart the srbd mpc
/// with a trajectory obeying LIPM dynamics

namespace dairlib::systems {

class LipmWarmStartSystem : public drake::systems::LeafSystem<double> {
 public:
  LipmWarmStartSystem(
      const multibody::SingleRigidBodyPlant &plant,
      double desired_com_height,
      const std::vector<int> &unordered_fsm_states,
      const std::vector<double> &unordered_state_durations,
      const std::vector<BipedStance> &unordered_state_stances);
  // Input port getters
  const drake::systems::InputPort<double> &get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

  const drake::systems::InputPort<double> &get_input_port_touchdown_time() const{
    return this->get_input_port(touchdown_time_port_);
  }

  const drake::systems::InputPort<double> &get_xdes_input_port() const {
    return this->get_input_port(x_des_input_port_);
  }

  // Output port getters
  const drake::systems::OutputPort<double> &get_output_port_lipm_from_current()
  const {
    return this->get_output_port(output_port_lipm_from_current_);
  }
  const drake::systems::OutputPort<double> &get_output_port_lipm_from_next()
  const {
    return this->get_output_port(output_port_lipm_from_next_);
  }
  const drake::systems::OutputPort<double> &get_output_port_lipm_from_final()
  const {
    return this->get_output_port(output_port_lipm_from_final_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructLipmTraj(const Eigen::VectorXd &CoM, const Eigen::VectorXd &dCoM,
                    const Eigen::VectorXd &stance_foot_pos, double start_time,
                    double end_time_of_this_fsm_state) const;

  void CalcTrajFromCurrent(const drake::systems::Context<double> &context,
                           drake::trajectories::Trajectory<double> *traj) const;
  void CalcTrajFromNext(const drake::systems::Context<double> &context,
                           drake::trajectories::Trajectory<double> *traj) const;
  void CalcTrajFromFinal(const drake::systems::Context<double> &context,
                           drake::trajectories::Trajectory<double> *traj) const;

  std::vector<std::vector<Eigen::Vector2d>> MakeDesXYVel(
      int n_step, double first_mode_duration,
      Eigen::VectorXd xdes, Eigen::VectorXd x) const;

  const int num_steps_ = 3;
  // Port indices
  int state_port_;
  int fsm_port_;
  int touchdown_time_port_;
  int x_des_input_port_;
  int output_port_lipm_from_current_;
  int output_port_lipm_from_next_;
  int output_port_lipm_from_final_;

  int mpc_input_sol_idx_;
  int mpc_state_sol_idx_;

  const multibody::SingleRigidBodyPlant &plant_;
  drake::systems::Context<double> *context_;

  double desired_com_height_;
  std::vector<int> unordered_fsm_states_;
  std::vector<double> unordered_state_durations_;
  std::vector<BipedStance> unordered_state_stances_;

};
}