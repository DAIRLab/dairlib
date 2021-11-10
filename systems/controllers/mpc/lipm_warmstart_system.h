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
      double stance_duration,
      double mpc_dt,
      const std::vector<int> &unordered_fsm_states,
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

  // Output port getters
  const drake::systems::OutputPort<double> &get_output_port_foot_target()
  const {
    return this->get_output_port(output_port_foot_target_);
  }
 private:
  void CalcWarmstartSolution(
      const drake::systems::Context<double> &context,
      drake::systems::BasicVector<double> *solvec) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructLipmTraj(const Eigen::VectorXd &CoM, const Eigen::VectorXd &dCoM,
                    const Eigen::VectorXd &stance_foot_pos, double start_time,
                    double end_time_of_this_fsm_state) const;


  void MakeCubicSrbdApproximationFromExponentials(
      std::vector<
          drake::trajectories::ExponentialPlusPiecewisePolynomial<double>> exps,
      drake::trajectories::Trajectory<double> *output_traj,
      double prev_touchdown_time) const;


  void CalcTrajFromCurrent(const drake::systems::Context<double> &context,
                           drake::trajectories::Trajectory<double> *traj) const;

  void CalcFootTarget(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const;


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
  int output_port_foot_target_;


  const drake::systems::CacheEntry* warmstart_sol_{};

  const multibody::SingleRigidBodyPlant &plant_;
  drake::systems::Context<double> *context_;

  double desired_com_height_;
  double stance_duration_;
  double mpc_dt_;
  std::vector<int> unordered_fsm_states_;
  std::vector<BipedStance> unordered_state_stances_;

};
}