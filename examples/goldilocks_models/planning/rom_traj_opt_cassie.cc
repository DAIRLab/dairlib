#include "examples/goldilocks_models/planning/rom_traj_opt_cassie.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/solvers/decision_variable.h"

#include "examples/goldilocks_models/planning/FoM_guard_constraint.h"
#include "examples/goldilocks_models/planning/FoM_reset_map_constraint.h"
#include "examples/goldilocks_models/planning/FoM_stance_foot_constraint.h"
#include "examples/goldilocks_models/planning/FoM_stride_length_constraint.h"
#include "examples/goldilocks_models/planning/dynamics_constraint.h"
#include "examples/goldilocks_models/planning/kinematics_constraint.h"

namespace dairlib {
namespace goldilocks_models {

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
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

RomTrajOptCassie::RomTrajOptCassie(
    vector<int> num_time_samples, vector<double> minimum_timestep,
    vector<double> maximum_timestep, MatrixXd Q, MatrixXd R,
    const ReducedOrderModel& rom, const MultibodyPlant<double>& plant,
    const StateMirror& state_mirror,
    const vector<pair<const Vector3d, const Frame<double>&>>& left_contacts,
    const vector<pair<const Vector3d, const Frame<double>&>>& right_contacts,
    const vector<std::tuple<std::string, double, double>>& fom_joint_name_lb_ub,
    VectorXd desired_final_position, VectorXd init_state, bool fix_all_timestep,
    bool zero_touchdown_impact)
    : MultipleShooting(
          rom.n_tau(), 2 * rom.n_y(),
          std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
              num_time_samples.size() + 1,
          1e-8, 1e8),
      num_modes_(num_time_samples.size()),
      mode_lengths_(num_time_samples),
      z_post_impact_vars_(NewContinuousVariables(
          (2 * rom.n_y()) * (num_time_samples.size() - 1), "zp")),
      x0_var_(NewContinuousVariables(
          (plant.num_positions() + plant.num_velocities()), "x0")),
      xf_vars_(NewContinuousVariables(
          (plant.num_positions() + plant.num_velocities()) *
              num_time_samples.size(),
          "xf")),
      v_post_impact_vars_(NewContinuousVariables(
          (plant.num_positions() + plant.num_velocities()) *
              (num_time_samples.size() - 1),
          "vp")),
      n_z_(2 * rom.n_y()),
      n_x_(plant.num_positions() + plant.num_velocities()),
      plant_(plant),
      rom_(rom) {
  DRAKE_ASSERT(minimum_timestep.size() == num_modes_);
  DRAKE_ASSERT(maximum_timestep.size() == num_modes_);

  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  int n_q = plant_.num_positions();
  int n_v = plant_.num_velocities();

  // Add cost
  cout << "Adding cost...\n";
  auto y = this->state();
  auto tau = this->input();
  this->AddRunningCost(y.tail(rom.n_y()).transpose() * Q * y.tail(rom.n_y()));
  this->AddRunningCost(tau.transpose() * R * tau);

  // Loop over modes to construct the problem (Initial guss and constraints)
  int counter = 0;
  for (int i = 0; i < num_modes_; i++) {
    cout << "Mode " << i << endl;
    mode_start_.push_back(counter);

    bool left_stance = i % 2 == 0;

    // Default initial guess to avoid singularity (which messes with gradient)
    for (int j = 0; j < mode_lengths_[i]; j++) {
      SetInitialGuess((state_vars_by_mode(i, j))(1), 1);
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
    auto dyn_constraint = std::make_shared<planning::DynamicsConstraint>(rom);
    DRAKE_ASSERT(static_cast<int>(dyn_constraint->num_constraints()) ==
                 num_states());
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      int time_index = mode_start_[i] + j;
      AddConstraint(
          dyn_constraint,
          {state_vars_by_mode(i, j),
           u_vars().segment(time_index * num_inputs(), num_inputs()),
           state_vars_by_mode(i, j + 1),
           u_vars().segment((time_index + 1) * num_inputs(), num_inputs()),
           h_vars().segment(time_index, 1)});
    }

    // Add RoM-FoM mapping constraints
    // TODO: might need to rotate the local frame to align with the global
    cout << "Adding RoM-FoM mapping constraint...\n";
    auto kin_constraint = std::make_shared<planning::KinematicsConstraint>(
        rom, plant, left_stance, state_mirror);
    auto z_0 = state_vars_by_mode(i, 0);
    auto z_f = state_vars_by_mode(i, mode_lengths_[i] - 1);
    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    AddConstraint(kin_constraint, {z_0, x_0});
    AddConstraint(kin_constraint, {z_f, x_f});

    // Add guard constraint
    cout << "Adding guard constraint...\n";
    const auto& swing_contacts = left_stance ? right_contacts : left_contacts;
    VectorXd lb_per_contact = VectorXd::Zero(2);
    if (!zero_touchdown_impact)
      lb_per_contact << 0, -std::numeric_limits<double>::infinity();
    VectorXd lb(2 * swing_contacts.size());
    for (int i = 0; i < swing_contacts.size(); i++) {
      lb.segment<2>(2 * i) = lb_per_contact;
    }
    VectorXd ub = VectorXd::Zero(2 * swing_contacts.size());
    auto guard_constraint = std::make_shared<planning::FomGuardConstraint>(
        plant, swing_contacts, lb, ub);
    AddConstraint(guard_constraint, xf_vars_by_mode(i));

    // Add (impact) discrete map constraint
    if (i != 0) {
      if (zero_touchdown_impact) {
        cout << "Adding (FoM velocity) identity reset map constraint...\n";
        AddLinearConstraint(xf_vars_by_mode(i - 1).segment(n_q, n_v) ==
                            x0_vars_by_mode(i).segment(n_q, n_v));
      } else {
        cout << "Adding (FoM velocity) reset map constraint...\n";
        auto reset_map_constraint =
            std::make_shared<planning::FomResetMapConstraint>(plant_,
                                                              swing_contacts);
        auto Lambda = NewContinuousVariables(3 * swing_contacts.size(),
                                             "Lambda" + to_string(i));
        AddConstraint(
            reset_map_constraint,
            {xf_vars_by_mode(i - 1),
             x0_vars_by_mode(i).tail(plant.num_velocities()), Lambda});
      }
    }

    // Full order model joint limits
    cout << "Adding full-order model joint constraint...\n";
    for (const auto& name_lb_ub : fom_joint_name_lb_ub) {
      AddBoundingBoxConstraint(
          std::get<1>(name_lb_ub), std::get<2>(name_lb_ub),
          x0_vars_by_mode(i)(positions_map.at(std::get<0>(name_lb_ub))));
      AddBoundingBoxConstraint(
          std::get<1>(name_lb_ub), std::get<2>(name_lb_ub),
          xf_vars_by_mode(i)(positions_map.at(std::get<0>(name_lb_ub))));
    }

    // Stitching x0 and xf (full-order model stance foot constraint)
    cout << "Adding full-order model stance foot constraint...\n";
    const auto& stance_contacts = left_stance ? left_contacts : right_contacts;
    auto fom_sf_constraint =
        std::make_shared<planning::FomStanceFootConstraint>(plant_,
                                                            stance_contacts);
    AddConstraint(fom_sf_constraint,
                  {x0_vars_by_mode(i).head(n_q), xf_vars_by_mode(i).head(n_q)});

    // Initial pose constraint for the full order model
    if (i == 0) {
      cout << "Adding initial pose constraint for full-order model...\n";
      AddBoundingBoxConstraint(init_state, init_state, x0_vars_by_mode(i));
      // AddLinearConstraint(x0_vars_by_mode(i)(0) == 0);
      cout << "init_state = " << init_state << endl;
    }

    // Final goal position constraint
    if (i == num_modes_ - 1) {
      cout << "Adding final position constraint for full-order model...\n";
      AddBoundingBoxConstraint(desired_final_position, desired_final_position,
                               xf_vars_by_mode(i).segment(4, 2));
    }

    // Stride length constraint
    // cout << "Adding stride length constraint for full-order model...\n";
    // V1
    // AddLinearConstraint(xf_vars_by_mode(i)(0) - x0_vars_by_mode(i)(0) ==
    // 0.304389); V2
    /*VectorXd stride_length(1); stride_length << 0.304389 * 2;
    auto fom_sl_constraint =
    std::make_shared<planning::FomStrideLengthConstraint>( left_stance, n_q,
    stride_length); AddConstraint(fom_sl_constraint,
    {x0_vars_by_mode(i).head(n_q), xf_vars_by_mode(i).head(n_q)
                                     });*/

    // Stride length cost
    /*if (i == num_modes_ - 1) {
      cout << "Adding final position cost for full-order model...\n";
      this->AddLinearCost(-10 * xf_vars_by_mode(i)(0));
    }*/

    counter += mode_lengths_[i] - 1;
  }
}

void RomTrajOptCassie::AddRegularizationCost(
    const Eigen::VectorXd& desired_final_position,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front, bool straight_leg_cost) {
  cout << "adding regularization cost ...\n";

  int n_q = plant_.num_positions();

  for (int i = 0; i < num_modes_; i++) {
    bool left_stance = i % 2 == 0;

    // Adding cost on FOM state increases convergence rate
    // If we only add position (not velocity) in the cost, then higher cost
    // results in spacing out each step more evenly
    MatrixXd Id_quat = 100 * MatrixXd::Identity(4, 4);
    MatrixXd Id_xy = 100 * MatrixXd::Identity(2, 2);
    MatrixXd Id_z_joints = 100 * MatrixXd::Identity(n_q - 6, n_q - 6);

    VectorXd modifixed_x_guess_left_in_front = x_guess_left_in_front;
    VectorXd modifixed_x_guess_right_in_front = x_guess_right_in_front;
    if (straight_leg_cost) {
      /*Id_periodic(5, 5) = 10;
      Id_periodic(6, 6) = 10;
      modifixed_x_guess_left_in_front(5) = 0;
      modifixed_x_guess_left_in_front(6) = 0;
      modifixed_x_guess_right_in_front(5) = 0;
      modifixed_x_guess_right_in_front(6) = 0;*/
    }

    if (left_stance) {
      this->AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_left_in_front.segment(2, n_q - 2),
          x0_vars_by_mode(i).segment(2, n_q - 2));
      this->AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_right_in_front.segment(2, n_q - 2),
          xf_vars_by_mode(i).segment(2, n_q - 2));
    } else {
      this->AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_right_in_front.segment(2, n_q - 2),
          x0_vars_by_mode(i).segment(2, n_q - 2));
      this->AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_left_in_front.segment(2, n_q - 2),
          xf_vars_by_mode(i).segment(2, n_q - 2));
    }
    this->AddQuadraticErrorCost(Id_xy, desired_final_position * i / num_modes_,
                                x0_vars_by_mode(i).head(1));
    this->AddQuadraticErrorCost(Id_xy,
                                desired_final_position * (i + 1) / num_modes_,
                                xf_vars_by_mode(i).head(1));
  }
}

void RomTrajOptCassie::SetAllInitialGuess(

    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& r_guess,
    const Eigen::MatrixXd& dr_guess, const Eigen::MatrixXd& tau_guess,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front,
    const Eigen::VectorXd& desired_final_position) {
  cout << "adding initial guess ...\n";

  MatrixXd y_guess(r_guess.rows() + dr_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  for (int i = 0; i < num_modes_; i++) {
    bool left_stance = i % 2 == 0;

    // Initial guess
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      SetInitialGuess(timestep(mode_start_[i] + j), h_guess.segment(1, 1));
    }
    for (int j = 0; j < mode_lengths_[i]; j++) {
      SetInitialGuess(state_vars_by_mode(i, j),
                      y_guess.block(0, j, 2 * rom_.n_y(), 1));
      int time_index = mode_start_[i] + j;
      SetInitialGuess(u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()),
                      tau_guess.col(j));
    }
    if (left_stance) {
      SetInitialGuess(x0_vars_by_mode(i).tail(n_x_ - 1),
                      x_guess_left_in_front.tail(n_x_ - 1));
      SetInitialGuess(xf_vars_by_mode(i).tail(n_x_ - 1),
                      x_guess_right_in_front.tail(
                          n_x_ - 1));  // TODO: this should be preimpact
    } else {
      SetInitialGuess(x0_vars_by_mode(i).tail(n_x_ - 1),
                      x_guess_right_in_front.tail(
                          n_x_ - 1));  // TODO: this should be preimpact
      SetInitialGuess(xf_vars_by_mode(i).tail(n_x_ - 1),
                      x_guess_left_in_front.tail(n_x_ - 1));
    }
    SetInitialGuess(x0_vars_by_mode(i).segment(4, 2),
                    desired_final_position * i / num_modes_);
    SetInitialGuess(xf_vars_by_mode(i).segment(4, 2),
                    desired_final_position * (i + 1) / num_modes_);
  }
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
RomTrajOptCassie::z_post_impact_vars_by_mode(int mode) const {
  return z_post_impact_vars_.segment(mode * n_z_, n_z_);
}
VectorXDecisionVariable RomTrajOptCassie::x0_vars_by_mode(int mode) const {
  if (mode == 0) {
    return x0_var_;
  } else {
    VectorXDecisionVariable ret(n_x_);
    ret << xf_vars_.segment(n_x_ * (mode - 1), plant_.num_positions()),
        v_post_impact_vars_.segment(plant_.num_velocities() * (mode - 1),
                                    plant_.num_velocities());
    return ret;
  }
}
const Eigen::VectorBlock<const VectorXDecisionVariable>
RomTrajOptCassie::xf_vars_by_mode(int mode) const {
  return xf_vars_.segment(mode * n_x_, n_x_);
}

VectorX<Expression> RomTrajOptCassie::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// RomTrajOptCassie::state_vars_by_mode(int mode, int
// time_index)  {
VectorXDecisionVariable RomTrajOptCassie::state_vars_by_mode(
    int mode, int time_index) const {
  if (time_index == 0 && mode > 0) {
    return z_post_impact_vars_by_mode(mode - 1);
  } else {
    VectorXDecisionVariable ret(num_states());
    return x_vars().segment((mode_start_[mode] + time_index) * num_states(),
                            num_states());
    // std::cout << Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0,
    // num_states())  << std::endl; return
    // Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0, num_states());
  }
}

// TODO: need to configure this to handle the hybrid discontinuities properly
void RomTrajOptCassie::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) * h_vars()(0) /
          2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

PiecewisePolynomial<double> RomTrajOptCassie::ReconstructInputTrajectory(
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

PiecewisePolynomial<double> RomTrajOptCassie::ReconstructStateTrajectory(
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
        times(k) += +1e-6;
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
  // return PiecewisePolynomial<double>::CubicHermite(times, states,
  // derivatives);
  return PiecewisePolynomial<double>::FirstOrderHold(times, states);
}

}  // namespace goldilocks_models
}  // namespace dairlib
