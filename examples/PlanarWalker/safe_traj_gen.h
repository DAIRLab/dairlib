#pragma once

#include <fstream>
#include <iostream>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "attic/multibody/rigidbody_utils.h"
#include "examples/PlanarWalker/LIPM_swing_leg.h"
#include "examples/PlanarWalker/create_lyapunov_polynomial.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib {

enum SteppingResults { dont_step, step, continue_balancing };

class SafeTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  SafeTrajGenerator(const RigidBodyTree<double>& tree,
                    const LIPMSwingLeg<double>& lipm_model,
                    LoadLyapunovPolynomial& polynomial_loader,
                    int left_foot_idx, Eigen::Vector3d pt_on_left_foot,
                    int right_foot_idx, Eigen::Vector3d pt_on_right_foot,
                    double mid_foot_height, double desired_final_foot_height,
                    double desired_final_vertical_foot_velocity,
                    bool add_extra_control);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

  Eigen::Vector3d solveQP(const Eigen::Vector3d& reduced_order_state) const;

  SteppingResults should_step(double current_time, double prev_td_time,
                   Eigen::Vector3d reduced_order_state) const;

  void find_next_stance_location(const Eigen::Vector3d& reduced_order_state,
                                 double& next_stance_loc,
                                 double& t) const;


 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  void CalcSwingTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::trajectories::PiecewisePolynomial<double> createSplineForSwingFoot(
      const double start_time_of_this_interval,
      const double end_time_of_this_interval,
      const Eigen::Vector3d& init_swing_foot_pos,
      const Eigen::Vector3d& CP) const;

  int state_port_;
  int fsm_port_;

  const RigidBodyTree<double>& tree_;
  const LIPMSwingLeg<double>& lipm_model_;
  LoadLyapunovPolynomial& polynomial_loader_;
  int left_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  int right_foot_idx_;
  Eigen::Vector3d pt_on_right_foot_;
  double mid_foot_height_;
  double desired_final_foot_height_;
  double desired_final_vertical_foot_velocity_;
  bool add_extra_control_;

  drake::systems::DiscreteStateIndex next_stance_foot_pos_idx_;
  drake::systems::DiscreteStateIndex last_calculation_time_idx_;
  drake::systems::DiscreteStateIndex duration_of_stance_idx_;

  int prev_td_time_idx_;
  int prev_fsm_state_idx_;
  int prev_td_swing_foot_idx_;

  bool is_quaternion_;

  const int left_stance_state_ = 0;
  const int right_stance_state_ = 1;

  Polynomiald V0_;
  Polynomiald W0_;
  Polynomiald V1_;
  Polynomiald W1_;

  std::vector<Polynomiald> partial_V0_;
  std::vector<Polynomiald> partial_V1_;

  std::vector<Polynomiald> x;

  // QP costs
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd S_;
  double dt_;

  // Mahematical Program
  std::unique_ptr<drake::solvers::MathematicalProgram> quadprog_;

  // Cost
  drake::solvers::QuadraticCost* quadcost_input_;
  drake::solvers::QuadraticCost* quadcost_acceleration_;
  drake::solvers::QuadraticCost* quadcost_swing_leg_;
  drake::solvers::LinearCost* lincost_rho_;

  // Constraints
  drake::solvers::LinearConstraint* input_constraint_;
  drake::solvers::LinearConstraint* barrier_constraint_;
  drake::solvers::LinearEqualityConstraint* acceleration_constraint_;

  // Variables
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      input_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      dx_;
  Eigen::Matrix<drake::symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>
      rho_;

  // Logging
  mutable std::ofstream logger_com_;
  mutable std::ofstream logger_swing_foot_;

  // Testing
  std::unique_ptr<std::vector<double>> time_hist_;
  std::unique_ptr<std::vector<double>> CoM_hist_x_;
  std::unique_ptr<std::vector<double>> CoM_hist_z_;
  std::unique_ptr<std::vector<double>> desired_CoM_hist_x_;
  std::unique_ptr<std::vector<double>> desired_CoM_hist_z_;
  std::unique_ptr<std::vector<double>> swing_pos_x_hist_;
  std::unique_ptr<std::vector<double>> swing_pos_y_hist_;
  std::unique_ptr<std::vector<double>> swing_pos_z_hist_;
  std::unique_ptr<std::vector<double>> desired_swing_pos_x_hist_;
  std::unique_ptr<std::vector<double>> desired_swing_pos_y_hist_;
  std::unique_ptr<std::vector<double>> desired_swing_pos_z_hist_;
  std::unique_ptr<std::vector<double>> stance_toe_angle_hist_;
  std::unique_ptr<std::vector<double>> swing_toe_angle_hist_;
};

} // namespace dairlib

