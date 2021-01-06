#include "examples/goldilocks_models/planning/rom_traj_opt.h"

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

// Note: In the zero foot impact case, adding post impact vel into decision
// variables (the so-called "slack variable") with linear constraint actually
// solves faster than without the variable. About 20% faster...

RomTrajOpt::RomTrajOpt(
    vector<int> num_time_samples, MatrixXd Q, MatrixXd R,
    const ReducedOrderModel& rom, const MultibodyPlant<double>& plant,
    const StateMirror& state_mirror, const vector<BodyPoint>& left_contacts,
    const vector<BodyPoint>& right_contacts,
    const vector<std::tuple<string, double, double>>& fom_joint_name_lb_ub,
    VectorXd x_init, bool start_with_left_stance, bool zero_touchdown_impact,
    bool print_status)
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
          plant.num_velocities() * (num_time_samples.size() - 1), "vp")),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      n_x_(plant.num_positions() + plant.num_velocities()),
      plant_(plant),
      rom_(rom),
      start_with_left_stance_(start_with_left_stance),
      print_status_(print_status) {
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  int n_q = plant_.num_positions();
  int n_v = plant_.num_velocities();

  // Add cost
  PrintStatus("Adding cost...");
  auto y = this->state();
  auto tau = this->input();
  this->AddRunningCost(y.tail(rom.n_y()).transpose() * Q * y.tail(rom.n_y()));
  this->AddRunningCost(tau.transpose() * R * tau);

  // Initial pose constraint for the full order model
  PrintStatus("Adding initial pose constraint for full-order model...");
  AddBoundingBoxConstraint(x_init, x_init, x0_vars_by_mode(0));
  // AddLinearConstraint(x0_vars_by_mode(i)(0) == 0);
  if (print_status_) {
    cout << "x_init = " << x_init.transpose() << endl;
  }

  // Loop over modes to add more constraints
  int counter = 0;
  bool left_stance = start_with_left_stance;
  for (int i = 0; i < num_modes_; i++) {
    PrintStatus("Mode " + std::to_string(i) + "============================");
    mode_start_.push_back(counter);

    // Add dynamics constraints at collocation points
    PrintStatus("Adding dynamics constraint...");
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
    PrintStatus("Adding RoM-FoM mapping constraint...");
    auto kin_constraint = std::make_shared<planning::KinematicsConstraint>(
        rom, plant, left_stance, state_mirror);
    auto z_0 = state_vars_by_mode(i, 0);
    auto z_f = state_vars_by_mode(i, mode_lengths_[i] - 1);
    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    AddConstraint(kin_constraint, {z_0, x_0});
    AddConstraint(kin_constraint, {z_f, x_f});

    // Add guard constraint
    PrintStatus("Adding guard constraint...");
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
        PrintStatus("Adding (FoM velocity) identity reset map constraint...");
        AddLinearConstraint(xf_vars_by_mode(i - 1).segment(n_q, n_v) ==
                            x0_vars_by_mode(i).segment(n_q, n_v));
      } else {
        PrintStatus("Adding (FoM velocity) reset map constraint...");
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
    PrintStatus("Adding full-order model joint constraint...");
    for (const auto& name_lb_ub : fom_joint_name_lb_ub) {
      if (i != 0) {
        // We don't impose constraint on the initial state (because it's
        // constrained already)
        AddBoundingBoxConstraint(
            std::get<1>(name_lb_ub), std::get<2>(name_lb_ub),
            x0_vars_by_mode(i)(positions_map.at(std::get<0>(name_lb_ub))));
      }
      AddBoundingBoxConstraint(
          std::get<1>(name_lb_ub), std::get<2>(name_lb_ub),
          xf_vars_by_mode(i)(positions_map.at(std::get<0>(name_lb_ub))));
    }

    // Stitching x0 and xf (full-order model stance foot constraint)
    PrintStatus("Adding full-order model stance foot constraint...");
    const auto& stance_contacts = left_stance ? left_contacts : right_contacts;
    auto fom_sf_constraint =
        std::make_shared<planning::FomStanceFootConstraint>(plant_,
                                                            stance_contacts);
    AddConstraint(fom_sf_constraint,
                  {x0_vars_by_mode(i).head(n_q), xf_vars_by_mode(i).head(n_q)});

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
    left_stance = !left_stance;
  }
}

void RomTrajOpt::AddTimeStepConstraint(std::vector<double> minimum_timestep,
                                       std::vector<double> maximum_timestep,
                                       bool fix_duration, double duration,
                                       bool equalize_timestep_size,
                                       double dt_0) {
  if (equalize_timestep_size && fix_duration) {
    if (dt_0 > 0) {
      double dt_value = (duration - dt_0) / (N() - 2);
      PrintStatus("Fix all timestep size (except the first one) to " +
                  to_string(dt_value));
      AddBoundingBoxConstraint(dt_0, dt_0, timestep(0));
      for (int i = 1; i < this->N() - 1; i++) {
        AddBoundingBoxConstraint(dt_value, dt_value, timestep(i));
      }
    } else {
      double dt_value = duration / (N() - 1);
      PrintStatus("Fix all timestep size to " + to_string(dt_value));
      for (int i = 0; i < this->N() - 1; i++) {
        AddBoundingBoxConstraint(dt_value, dt_value, timestep(i));
      }
    }
  } else {
    // Duration bound
    if (fix_duration) {
      PrintStatus("Fix time duration: total duration = " + to_string(duration));
      AddDurationBounds(duration, duration);
    }

    for (int i = 0; i < num_modes_; i++) {
      // Set timestep bounds
      for (int j = 0; j < mode_lengths_[i] - 1; j++) {
        AddBoundingBoxConstraint(minimum_timestep[i], maximum_timestep[i],
                                 timestep(mode_start_[i] + j));
      }

      // all timesteps within a mode must be equal
      for (int j = 0; j < mode_lengths_[i] - 2; j++) {
        AddLinearConstraint(timestep(mode_start_[i] + j) ==
                            timestep(mode_start_[i] + j + 1));
      }
    }

    // Make the timesteps between modes the same
    if (equalize_timestep_size) {
      PrintStatus("Equalize time steps between modes");
      for (int i = 1; i < num_modes_; i++) {
        if (mode_start_[i] > 0) {
          if (i == 1) {
            if (dt_0 <= 0) {
              AddLinearConstraint(timestep(mode_start_[i] - 1) ==
                                  timestep(mode_start_[i]));
            }
          } else {
            AddLinearConstraint(timestep(mode_start_[i] - 1) ==
                                timestep(mode_start_[i]));
          }
        }
      }
    }
  }
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
RomTrajOpt::z_post_impact_vars_by_mode(int mode) const {
  return z_post_impact_vars_.segment(mode * n_z_, n_z_);
}
VectorXDecisionVariable RomTrajOpt::x0_vars_by_mode(int mode) const {
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
RomTrajOpt::xf_vars_by_mode(int mode) const {
  return xf_vars_.segment(mode * n_x_, n_x_);
}

VectorX<Expression> RomTrajOpt::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// RomTrajOpt::state_vars_by_mode(int mode, int
// time_index)  {
VectorXDecisionVariable RomTrajOpt::state_vars_by_mode(int mode,
                                                       int time_index) const {
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
void RomTrajOpt::DoAddRunningCost(const drake::symbolic::Expression& g) {
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

void RomTrajOpt::GetStateAndDerivativeSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::MatrixXd>* derivative_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const {
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(derivative_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());

  VectorXd times(GetSampleTimes(result));

  for (int i = 0; i < num_modes_; i++) {
    MatrixXd states_i(num_states(), mode_lengths_[i]);
    MatrixXd derivatives_i(num_states(), mode_lengths_[i]);
    VectorXd times_i(mode_lengths_[i]);
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k_data = mode_start_[i] + j;

      VectorX<double> zk = result.GetSolution(state_vars_by_mode(i, j));
      VectorX<double> tauk = result.GetSolution(input(k_data));

      // z = [y; ydot]
      // Calculate zdot.
      // Copied from: examples/goldilocks_models/planning/dynamics_constraint.h
      VectorX<double> zdot(n_z_);
      zdot << zk.tail(n_y_),
          rom_.EvalDynamicFunc(zk.head(n_y_), zk.tail(n_y_), tauk);

      states_i.col(j) = drake::math::DiscardGradient(zk);
      derivatives_i.col(j) = drake::math::DiscardGradient(zdot);
      times_i(j) = times(k_data);
    }
    state_samples->push_back(states_i);
    derivative_samples->push_back(derivatives_i);
    state_breaks->push_back(times_i);
  }
}

PiecewisePolynomial<double> RomTrajOpt::ReconstructInputTrajectory(
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

PiecewisePolynomial<double> RomTrajOpt::ReconstructStateTrajectory(
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

RomTrajOptCassie::RomTrajOptCassie(
    vector<int> num_time_samples, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant,
    const StateMirror& state_mirror, const vector<BodyPoint>& left_contacts,
    const vector<BodyPoint>& right_contacts,
    const vector<std::tuple<string, double, double>>& fom_joint_name_lb_ub,
    Eigen::VectorXd x_init, bool start_with_left_stance,
    bool zero_touchdown_impact, bool print_status)
    : RomTrajOpt(num_time_samples, Q, R, rom, plant, state_mirror,
                 left_contacts, right_contacts, fom_joint_name_lb_ub, x_init,
                 start_with_left_stance, zero_touchdown_impact, print_status) {}

void RomTrajOptCassie::AddRegularizationCost(
    const Eigen::VectorXd& final_position,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front, bool straight_leg_cost) {
  PrintStatus("Adding regularization cost ...");

  int n_q = plant_.num_positions();

  bool left_stance = start_with_left_stance_;
  for (int i = 0; i < num_modes_; i++) {
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

    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    if (left_stance) {
      if (i != 0) {
        AddQuadraticErrorCost(
            Id_z_joints, modifixed_x_guess_left_in_front.segment(6, n_q - 6),
            x_0.segment(6, n_q - 6));
      }
      AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_right_in_front.segment(6, n_q - 6),
          x_f.segment(6, n_q - 6));
      //      AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_left_in_front.head(4),
      //          x_0.head(4));
      //      AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_right_in_front.head(4),
      //          x_f.head(4));
    } else {
      if (i != 0) {
        AddQuadraticErrorCost(
            Id_z_joints, modifixed_x_guess_right_in_front.segment(6, n_q - 6),
            x_0.segment(6, n_q - 6));
      }
      AddQuadraticErrorCost(Id_z_joints,
                            modifixed_x_guess_left_in_front.segment(6, n_q - 6),
                            x_f.segment(6, n_q - 6));
      //      AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_right_in_front.head(4),
      //          x_0.head(4));
      //      AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_left_in_front.head(4),
      //          x_f.head(4));
    }
    if (i != 0) {
      AddQuadraticErrorCost(Id_xy, final_position * i / num_modes_,
                            x_0.segment<2>(4));
    }
    AddQuadraticErrorCost(Id_xy, final_position * (i + 1) / num_modes_,
                          x_f.segment<2>(4));
    VectorX<double> quat_identity(4);
    quat_identity << 1, 0, 0, 0;
    if (i != 0) {
      AddQuadraticErrorCost(Id_quat, quat_identity, x_0.head(4));
    }
    AddQuadraticErrorCost(Id_quat, quat_identity, x_f.head(4));

    left_stance = !left_stance;
  }

  // Note: Cassie can exploit the "one-contact per foot" constraint to lean
  // forward at the end pose, so we add a hard constraint on quaternion here
  PrintStatus("Adding quaternion constraint on the final pose...");
  VectorX<double> quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  AddBoundingBoxConstraint(quat_identity, quat_identity,
                           xf_vars_by_mode(num_modes_ - 1).head(4));
}

void RomTrajOptCassie::SetAllInitialGuess(
    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& r_guess,
    const Eigen::MatrixXd& dr_guess, const Eigen::MatrixXd& tau_guess,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front,
    const Eigen::VectorXd& final_position, int fisrt_mode_phase_index) {
  PrintStatus("Adding initial guess ...");

  MatrixXd y_guess(r_guess.rows() + dr_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  bool left_stance = start_with_left_stance_;
  for (int i = 0; i < num_modes_; i++) {
    // Time steps
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      SetInitialGuess(timestep(mode_start_[i] + j), h_guess.segment(1, 1));
    }
    // Rom states and inputs
    for (int j = 0; j < mode_lengths_[i]; j++) {
      // The intial state might start in the middle of the stride
      if (i == 0) {
        SetInitialGuess(state_vars_by_mode(i, j),
                        y_guess.col(fisrt_mode_phase_index + j));
        int time_index = mode_start_[i] + j;
        SetInitialGuess(
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()),
            tau_guess.col(fisrt_mode_phase_index + j));
      } else {
        SetInitialGuess(state_vars_by_mode(i, j), y_guess.col(j));
        int time_index = mode_start_[i] + j;
        SetInitialGuess(
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()),
            tau_guess.col(j));
      }
    }
    // FOM states
    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    if (left_stance) {
      SetInitialGuess(x_0.tail(n_x_ - 6), x_guess_left_in_front.tail(n_x_ - 6));
      // TODO: this should be preimpact
      SetInitialGuess(x_f.tail(n_x_ - 6),
                      x_guess_right_in_front.tail(n_x_ - 6));
    } else {
      // TODO: this should be preimpact
      SetInitialGuess(x_0.tail(n_x_ - 6),
                      x_guess_right_in_front.tail(n_x_ - 6));
      SetInitialGuess(x_f.tail(n_x_ - 6), x_guess_left_in_front.tail(n_x_ - 6));
    }
    SetInitialGuess(x_0.segment(4, 2), final_position * i / num_modes_);
    SetInitialGuess(x_f.segment(4, 2), final_position * (i + 1) / num_modes_);
    VectorX<double> quat_identity(4);
    quat_identity << 1, 0, 0, 0;
    SetInitialGuess(x_0.head(4), quat_identity);
    SetInitialGuess(x_f.head(4), quat_identity);

    left_stance = !left_stance;
  }
}

void RomTrajOptCassie::AddRomRegularizationCost(
    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& r_guess,
    const Eigen::MatrixXd& dr_guess, const Eigen::MatrixXd& tau_guess,
    int fisrt_mode_phase_index, double w_reg) {
  PrintStatus("Adding regularization cost for ROM state ...");

  MatrixXd y_guess(r_guess.rows() + dr_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  MatrixXd I_h = w_reg * MatrixXd::Identity(1, 1);
  MatrixXd I_z = w_reg * MatrixXd::Identity(n_z_, n_z_);
  MatrixXd I_tau = w_reg * MatrixXd::Identity(rom_.n_tau(), rom_.n_tau());

  for (int i = 0; i < num_modes_; i++) {
    // Time steps
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      AddQuadraticErrorCost(I_h, h_guess.segment(1, 1),
                            timestep(mode_start_[i] + j));
    }
    // Rom states and inputs
    for (int j = 0; j < mode_lengths_[i]; j++) {
      // The intial state might start in the middle of the stride
      if (i == 0) {
        AddQuadraticErrorCost(I_z, y_guess.col(fisrt_mode_phase_index + j),
                              state_vars_by_mode(i, j));
        int time_index = mode_start_[i] + j;
        AddQuadraticErrorCost(
            I_tau, tau_guess.col(fisrt_mode_phase_index + j),
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()));
      } else {
        AddQuadraticErrorCost(I_z, y_guess.col(j), state_vars_by_mode(i, j));
        int time_index = mode_start_[i] + j;
        AddQuadraticErrorCost(
            I_tau, tau_guess.col(j),
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()));
      }
    }
  }
}

RomTrajOptFiveLinkRobot::RomTrajOptFiveLinkRobot(
    vector<int> num_time_samples, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant,
    const StateMirror& state_mirror, const vector<BodyPoint>& left_contacts,
    const vector<BodyPoint>& right_contacts,
    const vector<std::tuple<string, double, double>>& fom_joint_name_lb_ub,
    Eigen::VectorXd x_init, bool start_with_left_stance,
    bool zero_touchdown_impact)
    : RomTrajOpt(num_time_samples, Q, R, rom, plant, state_mirror,
                 left_contacts, right_contacts, fom_joint_name_lb_ub, x_init,
                 start_with_left_stance, zero_touchdown_impact) {}

void RomTrajOptFiveLinkRobot::AddRegularizationCost(
    const Eigen::VectorXd& final_position,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front, bool straight_leg_cost) {
  cout << "Adding regularization cost ...\n";

  int n_q = plant_.num_positions();

  bool left_stance = start_with_left_stance_;
  for (int i = 0; i < num_modes_; i++) {
    // Adding cost on FOM state increases convergence rate
    // If we only add position (not velocity) in the cost, then higher cost
    // results in spacing out each step more evenly
    MatrixXd Id_7 = 100 * MatrixXd::Identity(n_q - 1, n_q - 1);
    // Id_7(1,1) = 10;
    MatrixXd Id_1 = 100 * MatrixXd::Identity(1, 1);

    double torso_lean_forward_angle = 0.1;
    VectorXd modifixed_x_guess_left_in_front = x_guess_left_in_front;
    // modifixed_x_guess_left_in_front(2) = torso_lean_forward_angle;
    VectorXd modifixed_x_guess_right_in_front = x_guess_right_in_front;
    // modifixed_x_guess_right_in_front(2) = torso_lean_forward_angle;
    if (straight_leg_cost) {
      Id_7(5, 5) = 10;
      Id_7(6, 6) = 10;
      modifixed_x_guess_left_in_front(5) = 0;
      modifixed_x_guess_left_in_front(6) = 0;
      modifixed_x_guess_right_in_front(5) = 0;
      modifixed_x_guess_right_in_front(6) = 0;
    }

    if (left_stance) {
      AddQuadraticErrorCost(
          Id_7, modifixed_x_guess_left_in_front.head(n_q).tail(n_q - 1),
          x0_vars_by_mode(i).head(n_q).tail(n_q - 1));
      AddQuadraticErrorCost(
          Id_7, modifixed_x_guess_right_in_front.head(n_q).tail(n_q - 1),
          xf_vars_by_mode(i).head(n_q).tail(n_q - 1));
    } else {
      AddQuadraticErrorCost(
          Id_7, modifixed_x_guess_right_in_front.head(n_q).tail(n_q - 1),
          x0_vars_by_mode(i).head(n_q).tail(n_q - 1));
      AddQuadraticErrorCost(
          Id_7, modifixed_x_guess_left_in_front.head(n_q).tail(n_q - 1),
          xf_vars_by_mode(i).head(n_q).tail(n_q - 1));
    }
    AddQuadraticErrorCost(Id_1,
                          VectorXd::Ones(1) * final_position * i / num_modes_,
                          x0_vars_by_mode(i).head(1));
    AddQuadraticErrorCost(
        Id_1, VectorXd::Ones(1) * final_position * (i + 1) / num_modes_,
        xf_vars_by_mode(i).head(1));

    left_stance = !left_stance;
  }
}

void RomTrajOptFiveLinkRobot::SetAllInitialGuess(
    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& r_guess,
    const Eigen::MatrixXd& dr_guess, const Eigen::MatrixXd& tau_guess,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front,
    const Eigen::VectorXd& final_position) {
  cout << "Adding initial guess ...\n";

  MatrixXd y_guess(r_guess.rows() + dr_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  bool left_stance = start_with_left_stance_;
  for (int i = 0; i < num_modes_; i++) {
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
    SetInitialGuess(x0_vars_by_mode(i)(0), final_position(0) * i / num_modes_);
    SetInitialGuess(xf_vars_by_mode(i)(0),
                    final_position(0) * (i + 1) / num_modes_);

    left_stance = !left_stance;
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
