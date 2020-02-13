#include "examples/goldilocks_models/planning/RoM_planning_traj_opt.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
#include <string>

#include "drake/solvers/decision_variable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/planning/kinematics_constraint.h"
#include "examples/goldilocks_models/planning/dynamics_constraint.h"
#include "examples/goldilocks_models/planning/FoM_guard_constraint.h"
#include "examples/goldilocks_models/planning/FoM_reset_map_constraint.h"
#include "examples/goldilocks_models/planning/FoM_stance_foot_constraint.h"
#include "examples/goldilocks_models/planning/FoM_stride_length_constraint.h"

namespace dairlib {
namespace goldilocks_models {

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;

using std::string;
using std::to_string;

RomPlanningTrajOptWithFomImpactMap::RomPlanningTrajOptWithFomImpactMap(
  vector<int> num_time_samples,
  vector<double> minimum_timestep,
  vector<double> maximum_timestep,
  MatrixXd Q,
  MatrixXd R,
  int n_r,
  int n_tau,
  MatrixXd B_tau,
  int n_feature_kin,
  int n_feature_dyn,
  const VectorXd & theta_kin,
  const VectorXd & theta_dyn,
  const MultibodyPlant<double>& plant,
  bool zero_touchdown_impact,
  double desired_final_position,
  VectorXd init_state,
  VectorXd h_guess,
  MatrixXd r_guess,
  MatrixXd dr_guess,
  MatrixXd tau_guess,
  VectorXd x_guess_left_in_front,
  VectorXd x_guess_right_in_front,
  bool with_init_guess,
  bool fix_duration,
  bool fix_all_timestep,
  bool add_x_pose_in_cost,
  bool straight_leg_cost,
  int robot_option) :
  MultipleShooting(n_tau,
                   2 * n_r,
                   std::accumulate(num_time_samples.begin(),
                                   num_time_samples.end(), 0) - num_time_samples.size() + 1, 1e-8, 1e8),
  num_modes_(num_time_samples.size()),
  mode_lengths_(num_time_samples),
  y_post_impact_vars_(NewContinuousVariables(
                        (2 * n_r) * (num_time_samples.size() - 1), "y_p")),
  x0_vars_(NewContinuousVariables(
             (plant.num_positions() + plant.num_velocities()) * num_time_samples.size(),
             "x0")),
  xf_vars_(NewContinuousVariables(
             (plant.num_positions() + plant.num_velocities()) * num_time_samples.size(),
             "xf")),
  n_r_(n_r),
  n_tau_(n_tau),
  n_y_(2 * n_r),
  n_x_(plant.num_positions() + plant.num_velocities()),
  plant_(plant) {

  DRAKE_ASSERT(minimum_timestep.size() == num_modes_);
  DRAKE_ASSERT(maximum_timestep.size() == num_modes_);

  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  int n_q = plant_.num_positions();

  MatrixXd y_guess(2 * r_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  // Add cost
  cout << "Adding cost...\n";
  auto y = this->state();
  auto tau = this->input();
  this->AddRunningCost(y.tail(n_r).transpose()* Q * y.tail(n_r));
  this->AddRunningCost(tau.transpose()* R * tau);
  if (!add_x_pose_in_cost) {
    // Since there are mulitple q that could be mapped to the same r, I penalize
    // on q so it get close to a certain configuration
    MatrixXd Id = MatrixXd::Identity(1, 1);
    VectorXd zero_1d_vec = VectorXd::Zero(1);
    for (int i = 0; i < num_modes_; i++) {
      this->AddQuadraticErrorCost(1 * Id, zero_1d_vec,
                                  xf_vars_by_mode(i).segment(2, 1));
    }
  }
  // for (int i = 0; i < num_modes_; i++) {
  //   this->AddQuadraticErrorCost(10*Id, zero_1d_vec, xf_vars_by_mode(i).segment(3, 1));
  // }
  // for (int i = 0; i < num_modes_; i++) {
  //   this->AddQuadraticErrorCost(10*Id, zero_1d_vec, xf_vars_by_mode(i).segment(4, 1));
  // }

  // Duration bound
  if (fix_duration) {
    AddDurationBounds(h_guess.tail(1)(0)*num_modes_, h_guess.tail(1)(0)*num_modes_);
  }

  // (Initial guess and constraint) Initialization is looped over the modes
  int counter = 0;
  for (int i = 0; i < num_modes_; i++) {
    cout << "Mode " << i << endl;
    mode_start_.push_back(counter);

    bool left_stance = (i % 2 == 0) ? true : false;

    // Adding cost on FOM state increases convergence rate
    // If we only add position (not velocity) in the cost, then higher cost results in spacing out each step more evenly
    if (add_x_pose_in_cost) {
      int nq_or_nx = n_q; //n_q or 2*n_q
      MatrixXd Id_7 = 100 * MatrixXd::Identity(nq_or_nx - 1, nq_or_nx - 1);
      // Id_7(1,1) = 10;
      MatrixXd Id_1 = 100 * MatrixXd::Identity(1, 1);

      double torso_lean_forward_angle = 0.1;
      VectorXd modifixed_x_guess_left_in_front = x_guess_left_in_front;
      // modifixed_x_guess_left_in_front(2) = torso_lean_forward_angle;
      VectorXd modifixed_x_guess_right_in_front = x_guess_right_in_front;
      // modifixed_x_guess_right_in_front(2) = torso_lean_forward_angle;
      if(straight_leg_cost) {
        Id_7(5,5) = 10;
        Id_7(6,6) = 10;
        modifixed_x_guess_left_in_front(5) = 0;
        modifixed_x_guess_left_in_front(6) = 0;
        modifixed_x_guess_right_in_front(5) = 0;
        modifixed_x_guess_right_in_front(6) = 0;
      }

      if (left_stance) {
        this->AddQuadraticErrorCost(Id_7, modifixed_x_guess_left_in_front.head(nq_or_nx).tail(nq_or_nx - 1),
                                    x0_vars_by_mode(i).head(nq_or_nx).tail(nq_or_nx - 1));
        this->AddQuadraticErrorCost(Id_7, modifixed_x_guess_right_in_front.head(nq_or_nx).tail(nq_or_nx - 1),
                                    xf_vars_by_mode(i).head(nq_or_nx).tail(nq_or_nx - 1));
      } else {
        this->AddQuadraticErrorCost(Id_7, modifixed_x_guess_right_in_front.head(nq_or_nx).tail(nq_or_nx - 1),
                                    x0_vars_by_mode(i).head(nq_or_nx).tail(nq_or_nx - 1));
        this->AddQuadraticErrorCost(Id_7, modifixed_x_guess_left_in_front.head(nq_or_nx).tail(nq_or_nx - 1),
                                    xf_vars_by_mode(i).head(nq_or_nx).tail(nq_or_nx - 1));
      }
      this->AddQuadraticErrorCost(Id_1,
                                  VectorXd::Ones(1) * desired_final_position * i / num_modes_,
                                  x0_vars_by_mode(i).head(1));
      this->AddQuadraticErrorCost(Id_1,
                                  VectorXd::Ones(1) * desired_final_position * (i + 1) / num_modes_,
                                  xf_vars_by_mode(i).head(1));
    }

    // Initial guess
    if (with_init_guess) {
      for (int j = 0; j < mode_lengths_[i] - 1; j++) {
        SetInitialGuess(timestep(mode_start_[i] + j), h_guess.segment(1, 1));
      }
      for (int j = 0; j < mode_lengths_[i]; j++) {
        SetInitialGuess(state_vars_by_mode(i, j),
                        y_guess.block(0, j, 2 * n_r, 1));
        int time_index = mode_start_[i] + j;
        SetInitialGuess(u_vars().segment(time_index * n_tau_, n_tau_),
                        tau_guess.col(j));
      }
      if (left_stance) {
        SetInitialGuess(x0_vars_by_mode(i).tail(2 * n_q - 1),
                        x_guess_left_in_front.tail(2 * n_q - 1));
        SetInitialGuess(xf_vars_by_mode(i).tail(2 * n_q - 1),
                        x_guess_right_in_front.tail(2 * n_q - 1));
      } else {
        SetInitialGuess(x0_vars_by_mode(i).tail(2 * n_q - 1),
                        x_guess_right_in_front.tail(2 * n_q - 1));
        SetInitialGuess(xf_vars_by_mode(i).tail(2 * n_q - 1),
                        x_guess_left_in_front.tail(2 * n_q - 1));
      }
      SetInitialGuess(x0_vars_by_mode(i)(0),
                      desired_final_position * i / num_modes_);
      SetInitialGuess(xf_vars_by_mode(i)(0),
                      desired_final_position * (i + 1) / num_modes_);
    } else {
      // Initial to avoid sigularity (which messes with gradient)
      for (int j = 0; j < mode_lengths_[i]; j++) {
        SetInitialGuess((state_vars_by_mode(i, j))(1), 1);
      }
    }

    // Constraints
    // Set timestep bounds
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      AddBoundingBoxConstraint(minimum_timestep[i], maximum_timestep[i],
                               timestep(mode_start_[i] + j));
    }
    for (int j = 0; j < mode_lengths_[i] - 2; j++) {
      // all timesteps must be equal
      AddLinearConstraint(timestep(mode_start_[i] + j) ==
                          timestep(mode_start_[i] + j + 1));
    }
    // make the timesteps in each mode the same
    if (fix_all_timestep && i != 0) {
      AddLinearConstraint(timestep(mode_start_[i] - 1) ==
                          timestep(mode_start_[i]));
    }


    // Add dynamics constraints at collocation points
    cout << "Adding dynamics constraint...\n";
    auto dyn_constraint = std::make_shared<planning::DynamicsConstraint>(
                            n_r, n_r, n_feature_dyn, theta_dyn, n_tau, B_tau, robot_option);
    DRAKE_ASSERT(
      static_cast<int>(dyn_constraint->num_constraints()) == num_states());
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      int time_index = mode_start_[i] + j;
      AddConstraint(dyn_constraint,
      { state_vars_by_mode(i, j),
        u_vars().segment(time_index * num_inputs(), num_inputs()),
        state_vars_by_mode(i, j + 1),
        u_vars().segment((time_index + 1) * num_inputs(), num_inputs()),
        h_vars().segment(time_index, 1)
      });
    }

    // Add RoM-FoM mapping constraints
    cout << "Adding RoM-FoM mapping constraint...\n";
    auto kin_constraint = std::make_shared<planning::KinematicsConstraint>(
                            n_r, n_q, n_feature_kin, theta_kin, robot_option);
    auto y_0 = state_vars_by_mode(i, 0);
    auto y_f = state_vars_by_mode(i, mode_lengths_[i] - 1);
    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    if (left_stance) {
      AddConstraint(kin_constraint, {y_0, x_0});
      AddConstraint(kin_constraint, {y_f, x_f});
    } else {
      VectorXDecisionVariable x_0_swap(n_x_);
      x_0_swap << x_0.segment(0, 3),
               x_0.segment(4, 1),
               x_0.segment(3, 1),
               x_0.segment(6, 1),
               x_0.segment(5, 1),
               x_0.segment(0 + n_q, 3),
               x_0.segment(4 + n_q, 1),
               x_0.segment(3 + n_q, 1),
               x_0.segment(6 + n_q, 1),
               x_0.segment(5 + n_q, 1);
      VectorXDecisionVariable x_f_swap(n_x_);
      x_f_swap << x_f.segment(0, 3),
               x_f.segment(4, 1),
               x_f.segment(3, 1),
               x_f.segment(6, 1),
               x_f.segment(5, 1),
               x_f.segment(0 + n_q, 3),
               x_f.segment(4 + n_q, 1),
               x_f.segment(3 + n_q, 1),
               x_f.segment(6 + n_q, 1),
               x_f.segment(5 + n_q, 1);
      AddConstraint(kin_constraint, {y_0, x_0_swap});
      AddConstraint(kin_constraint, {y_f, x_f_swap});
    }

    // Add guard constraint
    cout << "Adding guard constraint...\n";
    VectorXd lb = VectorXd::Zero(2);
    if (!zero_touchdown_impact)
      lb << 0, -std::numeric_limits<double>::infinity();
    VectorXd ub = VectorXd::Zero(2);
    auto guard_constraint = std::make_shared<planning::FomGuardConstraint>(
                              left_stance, n_q, n_q, lb, ub);
    AddConstraint(guard_constraint, xf_vars_by_mode(i));

    // Add constraints for stitching FOM positins
    if (i != 0) {
      cout << "Adding (FoM position) periodicity constraint...\n";
      AddLinearConstraint(xf_vars_by_mode(i - 1).segment(0, n_q) ==
                          x0_vars_by_mode(i).segment(0, n_q));
    }

    // Add (impact) discrete map constraint
    if (i != 0) {
      if (zero_touchdown_impact) {
        cout << "Adding (FoM velocity) identity reset map constraint...\n";
        AddLinearConstraint(xf_vars_by_mode(i - 1).segment(n_q, n_q) ==
                            x0_vars_by_mode(i).segment(n_q, n_q));
      } else {
        cout << "Adding (FoM velocity) reset map constraint...\n";
        int n_J = 2;
        auto reset_map_constraint =
          std::make_shared<planning::FomResetMapConstraint>(
            left_stance, n_q, n_q, n_J, plant_);
        auto Lambda = NewContinuousVariables(n_J, "Lambda" + to_string(i));
        AddConstraint(reset_map_constraint, {xf_vars_by_mode(i - 1),
                                             x0_vars_by_mode(i),
                                             Lambda
                                            });
      }
    }

    // Full order model joint limits
    cout << "Adding full-order model joint constraint...\n";
    vector<string> l_or_r{"left_", "right_"};
    vector<string> fom_joint_names{"hip_pin", "knee_pin"};
    vector<double> lb_for_fom_joints{ -M_PI / 2.0, 5.0 / 180.0 * M_PI};
    vector<double> ub_for_fom_joints{ M_PI / 2.0, M_PI / 2.0};
    for (unsigned int k = 0; k < l_or_r.size(); k++) {
      for (unsigned int l = 0; l < fom_joint_names.size(); l++) {
        AddLinearConstraint(
          x0_vars_by_mode(i)(positions_map.at(l_or_r[k] + fom_joint_names[l])),
          lb_for_fom_joints[l], ub_for_fom_joints[l]);
        AddLinearConstraint(
          xf_vars_by_mode(i)(positions_map.at(l_or_r[k] + fom_joint_names[l])),
          lb_for_fom_joints[l], ub_for_fom_joints[l]);
      }
    }

    // Sitching x0 and xf (full-order model stance foot constraint)
    cout << "Adding full-order model stance foot constraint...\n";
    auto fom_sf_constraint = std::make_shared<planning::FomStanceFootConstraint>(
                               left_stance, n_q);
    AddConstraint(fom_sf_constraint, {x0_vars_by_mode(i).head(n_q),
                                      xf_vars_by_mode(i).head(n_q)
                                     });

    // Initial pose constraint for the full order model
    if (i == 0) {
      cout << "Adding initial pose constraint for full-order model...\n";
      AddLinearConstraint(x0_vars_by_mode(i) == init_state);
      // AddLinearConstraint(x0_vars_by_mode(i)(0) == 0);
      cout << "init_state = " << init_state << endl;
    }

    // Stride length constraint
    if (i == num_modes_ - 1) {
      cout << "Adding final position constraint for full-order model...\n";
      AddLinearConstraint(xf_vars_by_mode(i)(0) == desired_final_position);
    }
    // cout << "Adding stride length constraint for full-order model...\n";
    // V1
    // AddLinearConstraint(xf_vars_by_mode(i)(0) - x0_vars_by_mode(i)(0) == 0.304389);
    // V2
    /*VectorXd stride_length(1); stride_length << 0.304389 * 2;
    auto fom_sl_constraint = std::make_shared<planning::FomStrideLengthConstraint>(
                               left_stance, n_q, stride_length);
    AddConstraint(fom_sl_constraint, {x0_vars_by_mode(i).head(n_q),
                                      xf_vars_by_mode(i).head(n_q)
                                     });*/

    // Stride length cost
    /*if (i == num_modes_ - 1) {
      cout << "Adding final position cost for full-order model...\n";
      this->AddLinearCost(-10 * xf_vars_by_mode(i)(0));
    }*/

    counter += mode_lengths_[i] - 1;
  }
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
RomPlanningTrajOptWithFomImpactMap::y_post_impact_vars_by_mode(
  int mode) const {
  return y_post_impact_vars_.segment(mode * n_y_, n_y_);
}
const Eigen::VectorBlock<const VectorXDecisionVariable>
RomPlanningTrajOptWithFomImpactMap::x0_vars_by_mode(int mode) const {
  return x0_vars_.segment(mode * n_x_, n_x_);
}
const Eigen::VectorBlock<const VectorXDecisionVariable>
RomPlanningTrajOptWithFomImpactMap::xf_vars_by_mode(int mode) const {
  return xf_vars_.segment(mode * n_x_, n_x_);
}

VectorX<Expression>
RomPlanningTrajOptWithFomImpactMap::SubstitutePlaceholderVariables(
  const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) = MultipleShooting::SubstitutePlaceholderVariables(f(i),
             interval_index);
  }
  return ret;
}


// Eigen::VectorBlock<const VectorXDecisionVariable> RomPlanningTrajOptWithFomImpactMap::state_vars_by_mode(int mode, int time_index)  {
VectorXDecisionVariable RomPlanningTrajOptWithFomImpactMap::state_vars_by_mode(
  int mode, int time_index) const {
  if (time_index == 0 && mode > 0) {
    return y_post_impact_vars_by_mode(mode - 1);
  } else {
    VectorXDecisionVariable ret(num_states());
    return x_vars().segment((mode_start_[mode] + time_index) * num_states(),
                            num_states());
    // std::cout << Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0, num_states())  << std::endl;
    // return Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0, num_states());
  }
}

//TODO: need to configure this to handle the hybrid discontinuities properly
void RomPlanningTrajOptWithFomImpactMap::DoAddRunningCost(
  const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) *
          h_vars()(0) / 2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g , i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

PiecewisePolynomial<double>
RomPlanningTrajOptWithFomImpactMap::ReconstructInputTrajectory(
  const MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  vector<double> times_vec(N());
  vector<Eigen::MatrixXd> inputs(N());
  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}

PiecewisePolynomial<double>
RomPlanningTrajOptWithFomImpactMap::ReconstructStateTrajectory(
  const MathematicalProgramResult& result) const {
  VectorXd times_all(GetSampleTimes(result));
  VectorXd times(N() + num_modes_ - 1);

  MatrixXd states(num_states(), N() + num_modes_ - 1);
  MatrixXd inputs(num_inputs(), N() + num_modes_ - 1);
  MatrixXd derivatives(num_states(), N() + num_modes_ - 1);

  for (int i = 0; i < num_modes_; i++) {
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k = mode_start_[i] + j + i;
      int k_data = mode_start_[i] + j;
      times(k) = times_all(k_data);

      // False timestep to match velocities
      if (i > 0 && j == 0) {
        times(k) += + 1e-6;
      }
      VectorX<double> yk = result.GetSolution(state_vars_by_mode(i, j));
      VectorX<double> tauk = result.GetSolution(input(k_data));
      states.col(k) = yk;
      inputs.col(k) = tauk;


      // TODO(yminchen): need to modify the following code
      /*auto context = multibody::createContext(plant_, yk, tauk);
      constraints_[i]->updateData(*context, result.GetSolution(force(i, j)));
      derivatives.col(k) =
        drake::math::DiscardGradient(constraints_[i]->getXDot());*/
    }
  }
  // return PiecewisePolynomial<double>::Cubic(times, states, derivatives);
  return PiecewisePolynomial<double>::FirstOrderHold(times, states);
}



}  // namespace goldilocks_models
}  // namespace dairlib
