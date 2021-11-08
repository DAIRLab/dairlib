#include "lipm_mpc_qp.h"
#include <math.h>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/solvers/decision_variable.h"

typedef std::numeric_limits<double> dbl;

namespace dairlib::systems {

using std::cout;
using std::endl;
using std::pair;
using std::string;
using std::to_string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::trajectories::PiecewisePolynomial;

LipmMpc::LipmMpc(const std::vector<Eigen::Vector2d>& des_xy_pos,
                 const std::vector<Eigen::Vector2d>& des_xy_vel,
                 double w_predict_lipm_p, double w_predict_lipm_v,
                 const Eigen::VectorXd& init_pos,
                 const Eigen::VectorXd& init_vel,
                 const Eigen::VectorXd& init_input, int n_step,
                 double first_mode_duration, double stride_period,
                 std::vector<double> height_vec, double max_length_foot_to_body,
                 double max_length_foot_to_body_front, double min_step_width,
                 bool start_with_left_stance)
    : x_lipm_vars_(NewContinuousVariables(4 * (n_step + 1), "x_lipm_vars")),
      u_lipm_vars_(NewContinuousVariables(2 * n_step, "u_lipm_vars")),
      n_step_(n_step) {
  DRAKE_DEMAND(n_step > 0);

  //
  bool left_stance = start_with_left_stance;

  // Initial state and input constraint
  AddBoundingBoxConstraint(init_pos, init_pos, x_lipm_vars_.head<2>());
  AddBoundingBoxConstraint(init_vel, init_vel, x_lipm_vars_.segment<2>(2));
  AddBoundingBoxConstraint(init_input, init_input, u_lipm_vars_.head<2>());

  // Add LIPM dynamics constraint
  vector<Eigen::Matrix<double, 2, 5>> A_lin_vec(n_step);
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);
  for (int i = 0; i < n_step; i++) {
    ConstructDynamicMatrices(height_vec.at(i),
                             (i == 0) ? first_mode_duration : stride_period, &A,
                             &B);
    A_lin_vec.at(i) << A, B, -I2;
  }

  for (int i = 0; i < n_step; i++) {
    VectorXDecisionVariable x_i = x_lipm_vars_by_idx(i);
    VectorXDecisionVariable u_i = u_lipm_vars_by_idx(i);
    VectorXDecisionVariable x_i_post = x_lipm_vars_by_idx(i + 1);
    // x axis
    AddLinearEqualityConstraint(
        A_lin_vec.at(i), VectorXd::Zero(2),
        {x_i.segment<1>(0), x_i.segment<1>(2), u_i.segment<1>(0),
         x_i_post.segment<1>(0), x_i_post.segment<1>(2)});
    // y axis
    AddLinearEqualityConstraint(
        A_lin_vec.at(i), VectorXd::Zero(2),
        {x_i.segment<1>(1), x_i.segment<1>(3), u_i.segment<1>(1),
         x_i_post.segment<1>(1), x_i_post.segment<1>(3)});
  }

  // Add step size kinematics constraint
  Eigen::Matrix<double, 1, 2> A_lin_kin;
  A_lin_kin << 1, -1;
  Eigen::Matrix<double, 1, 1> ub_x;
  ub_x
      << max_length_foot_to_body_front;  // note that the constraint is wrt foot
  Eigen::Matrix<double, 1, 1> lb_x;
  lb_x << -max_length_foot_to_body;
  Eigen::Matrix<double, 1, 1> ub_y;
  ub_y << max_length_foot_to_body;
  Eigen::Matrix<double, 1, 1> lb_y;
  lb_y << min_step_width;

  for (int i = 1; i <= n_step; i++) {
    VectorXDecisionVariable x_i = x_lipm_vars_by_idx(i);
    // 1. end of mode
    VectorXDecisionVariable u_i = u_lipm_vars_by_idx(i - 1);
    AddLinearConstraint(A_lin_kin, lb_x, ub_x,
                        {x_i.segment<1>(0), u_i.head<1>()});
    AddLinearConstraint(A_lin_kin, left_stance ? -ub_y : lb_y,
                        left_stance ? -lb_y : ub_y,
                        {x_i.segment<1>(1), u_i.tail<1>()});

    // 2. start of next mode
    if (i != n_step) {
      VectorXDecisionVariable u_i_post = u_lipm_vars_by_idx(i);
      AddLinearConstraint(A_lin_kin, lb_x, ub_x,
                          {x_i.segment<1>(0), u_i_post.head<1>()});
      AddLinearConstraint(A_lin_kin, left_stance ? lb_y : -ub_y,
                          left_stance ? ub_y : -lb_y,
                          {x_i.segment<1>(1), u_i_post.tail<1>()});
    }
    left_stance = !left_stance;
  }

  // Add cost
  //  cout << "add lipm tracking\n des_xy_pos = \n";
  for (int i = 0; i < n_step; i++) {
    //    cout << des_xy_pos.at(i + 1).transpose() << endl;
    lipm_p_bindings_.push_back(
        AddQuadraticErrorCost(w_predict_lipm_p * I2, des_xy_pos.at(i + 1),
                              x_lipm_vars_by_idx(i + 1).head<2>()));
    lipm_v_bindings_.push_back(
        AddQuadraticErrorCost(w_predict_lipm_v * I2, des_xy_vel.at(i),
                              x_lipm_vars_by_idx(i + 1).tail<2>()));
  }
}

Eigen::MatrixXd LipmMpc::GetStateSamples(
    const drake::solvers::MathematicalProgramResult& result) const {
  VectorX<double> state_sol = result.GetSolution(x_lipm_vars_);
  return Eigen::Map<MatrixXd>(state_sol.data(), 4, n_step_ + 1);
}
Eigen::MatrixXd LipmMpc::GetInputSamples(
    const drake::solvers::MathematicalProgramResult& result) const {
  VectorX<double> input_sol = result.GetSolution(u_lipm_vars_);
  return Eigen::Map<MatrixXd>(input_sol.data(), 2, n_step_);
}

void LipmMpc::ConstructDynamicMatrices(double height, double stride_period,
                                       Eigen::Matrix<double, 2, 2>* A,
                                       Eigen::Matrix<double, 2, 1>* B) {
  double omega = std::sqrt(9.81 / height);
  double cosh_wT = std::cosh(omega * stride_period);
  double sinh_wT = std::sinh(omega * stride_period);
  (*A) << cosh_wT, sinh_wT / omega, omega * sinh_wT, cosh_wT;
  (*B) << 1 - cosh_wT, -omega * sinh_wT;
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
LipmMpc::x_lipm_vars_by_idx(int idx) const {
  return x_lipm_vars_.segment(idx * 4, 4);
}
const Eigen::VectorBlock<const VectorXDecisionVariable>
LipmMpc::u_lipm_vars_by_idx(int idx) const {
  return u_lipm_vars_.segment(idx * 2, 2);
}

}  // namespace dairlib::systems
