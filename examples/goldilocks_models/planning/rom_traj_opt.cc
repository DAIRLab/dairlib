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
          (2 * rom.n_y()) * (num_time_samples.size() - 1), "xp")),
      x0_var_(NewContinuousVariables(
          (plant.num_positions() + plant.num_velocities()), "x0_FOM")),
      xf_vars_(NewContinuousVariables(
          (plant.num_positions() + plant.num_velocities()) *
              num_time_samples.size(),
          "xf_FOM")),
      v_post_impact_vars_(NewContinuousVariables(
          plant.num_velocities() * (num_time_samples.size() - 1), "vp_FOM")),
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
  bool soft_init_constraint = true;
  if (soft_init_constraint) {
    VectorXDecisionVariable x0 = x0_vars_by_mode(0);
    /// relax the state
    /* PrintStatus("(relax the whole state)");
    auto eps = NewContinuousVariables(n_x_, "eps_x0_FOM");
    MatrixXd Aeq = MatrixXd::Ones(1, 2);
    for (int i = 0; i < n_x_; i++) {
      AddLinearEqualityConstraint(
          Aeq, x_init.segment<1>(i),
          {x0.segment<1>(i), eps.segment<1>(i)});
    }
    MatrixXd Q_x0 = 100 * MatrixXd::Identity(n_x_, n_x_);
    VectorXd b_x0 = VectorXd::Zero(n_x_);
    AddQuadraticCost(Q_x0, b_x0, eps);
    SetInitialGuess(eps, VectorXd::Zero(n_x_));*/
    /// relax only the velocity
    // TODO: not sure why the runtime is so slow. maybe tune Q_v0?
    PrintStatus("(relax only the velocity)");
    int start_idx_v_relax = 3;
    int len_v_relax = 3;
    auto eps = NewContinuousVariables(len_v_relax, "eps_v0_FOM");
    const VectorXDecisionVariable& v0_float_vars =
        x0.segment(n_q + start_idx_v_relax, len_v_relax);
    const VectorXd v_init =
        x_init.segment(n_q + start_idx_v_relax, len_v_relax);
    MatrixXd Aeq = MatrixXd::Ones(1, 2);
    for (int i = 0; i < len_v_relax; i++) {
      AddLinearEqualityConstraint(
          Aeq, v_init.segment<1>(i),
          {v0_float_vars.segment<1>(i), eps.segment<1>(i)});
    }
    MatrixXd Q_v0 = 1 * MatrixXd::Identity(len_v_relax, len_v_relax);
    VectorXd b_v0 = VectorXd::Zero(len_v_relax);
    AddQuadraticCost(Q_v0, b_v0, eps);
    SetInitialGuess(eps, 0 * VectorXd::Ones(len_v_relax));
    // The rest of the state should be hard-constrained
    AddBoundingBoxConstraint(x_init.head(n_q + start_idx_v_relax),
                             x_init.head(n_q + start_idx_v_relax),
                             x0.head(n_q + start_idx_v_relax));
    AddBoundingBoxConstraint(
        x_init.segment(n_q + start_idx_v_relax + len_v_relax,
                       n_v - start_idx_v_relax - len_v_relax),
        x_init.segment(n_q + start_idx_v_relax + len_v_relax,
                       n_v - start_idx_v_relax - len_v_relax),
        x0.segment(n_q + start_idx_v_relax + len_v_relax,
                   n_v - start_idx_v_relax - len_v_relax));
  } else {
    AddBoundingBoxConstraint(x_init, x_init, x0_vars_by_mode(0));
    // AddLinearConstraint(x0_vars_by_mode(i)(0) == 0);
  }
  if (print_status_) {
    cout << "x_init = " << x_init.transpose() << endl;
  }

  // Loop over modes to add more constraints
  int counter = 0;
  bool left_stance = start_with_left_stance;
  for (int i = 0; i < num_modes_; i++) {
    PrintStatus("Mode " + std::to_string(i) + "============================");
    mode_start_.push_back(counter);

    VectorXDecisionVariable x0 = x0_vars_by_mode(i);
    VectorXDecisionVariable xf = xf_vars_by_mode(i);

    // Testing -- penalize the velocity of the FOM states (help to
    // regularize)
    //    MatrixXd Q_v_FOM = 0.01 * MatrixXd::Identity(n_v, n_v);
    //    VectorXd b_v_FOM = VectorXd::Zero(n_v);
    // Version 1
    //    if (i != 0) {
    //      AddQuadraticCost(Q_v_FOM, b_v_FOM, x0.tail(n_v));
    //    }
    //    AddQuadraticCost(Q_v_FOM, b_v_FOM, xf.tail(n_v));
    // Version 2
    //    if (i == num_modes_ - 1) {
    //      AddQuadraticCost(Q_v_FOM, b_v_FOM, xf.tail(n_v));
    //    }

    // Add dynamics constraints at collocation points
    PrintStatus("Adding dynamics constraint...");
    auto dyn_constraint = std::make_shared<planning::DynamicsConstraint>(rom);
    DRAKE_DEMAND(static_cast<int>(dyn_constraint->num_constraints()) ==
                 num_states());
    dyn_constraint->SetConstraintScaling(rom_dyn_constraint_scaling_);
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
    kin_constraint->SetConstraintScaling(rom_fom_mapping_constraint_scaling_);
    VectorXDecisionVariable z_0 = state_vars_by_mode(i, 0);
    VectorXDecisionVariable z_f = state_vars_by_mode(i, mode_lengths_[i] - 1);
    AddConstraint(kin_constraint, {z_0, x0});
    AddConstraint(kin_constraint, {z_f, xf});

    // Testing state mirroring
    //    if (!left_stance) {
    //      VectorXd x = VectorXd::Zero(n_z_ + n_x_);
    //      x.tail(n_x_) << 0.95800512, 0.01326131, 0.10208891, 0.02493059,
    //          -0.00357683, -0.07077365, 1.00229253, 0.06585518, 0.08007648,
    //          -0.03271629, 0.04345352, 0.30198447, 0.49940965, -0.646,
    //          -0.74489845, 1.40071356, 1.36480139, -1.67787617, -1.78524973,
    //          0.2511753, -0.48393843, 0.19170832, -0.14228517, -0.55363658,
    //          0.00499403, 0.33995008, 0.37624165, -0.14535965, -0.17715706,
    //          -0.28741955, -0.62009629, 0.09105082, 0.35101321, -0.26117723,
    //          -0.127291, 0.00350424, -0.09447938;
    //      VectorXd y = VectorXd::Zero(n_z_);
    //      kin_constraint.get()->Eval(x, &y);
    //      DRAKE_DEMAND(false);
    //    }

    // Add guard constraint
    PrintStatus("Adding guard constraint...");
    const auto& swing_contacts = left_stance ? right_contacts : left_contacts;
    //    if (i != num_modes_ - 1) {
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
    guard_constraint->SetConstraintScaling(fom_guard_constraint_scaling_);
    AddConstraint(guard_constraint, xf);
    //    }

    // Add (impact) discrete map constraint
    if (i != 0) {
      VectorXDecisionVariable xf_prev = xf_vars_by_mode(i - 1);
      if (zero_touchdown_impact) {
        PrintStatus("Adding (FoM velocity) identity reset map constraint...");
        AddLinearConstraint(xf_prev.segment(n_q, n_v) == x0.segment(n_q, n_v));
      } else {
        PrintStatus("Adding (FoM velocity) reset map constraint...");
        auto reset_map_constraint =
            std::make_shared<planning::FomResetMapConstraint>(plant_,
                                                              swing_contacts);
        reset_map_constraint->SetConstraintScaling(
            fom_discrete_dyn_constraint_scaling_);
        int n_Lambda = 3 * swing_contacts.size();
        auto Lambda =
            NewContinuousVariables(n_Lambda, "Lambda_FOM_" + to_string(i));
        AddConstraint(reset_map_constraint, {xf_prev, x0.tail(n_v), Lambda});
        // Regularization term (there is a 1DoF null space in the force)
        //  MatrixXd Q_lambda = 0.01 * MatrixXd::Identity(n_Lambda, n_Lambda);
        //  VectorXd b_lambda = VectorXd::Zero(n_Lambda);
        //  AddQuadraticCost(Q_lambda, b_lambda, Lambda);

        // debugging
//        lambda_cost_bindings_.push_back(
//            AddLinearCost(10 * Lambda(2) + 10 * Lambda(5)));
      }
    }

    // Full order model joint limits
    PrintStatus("Adding full-order model joint constraint...");
    for (const auto& name_lb_ub : fom_joint_name_lb_ub) {
      if (i != 0) {
        // We don't impose constraint on the initial state (because it's
        // constrained already)
        AddBoundingBoxConstraint(std::get<1>(name_lb_ub),
                                 std::get<2>(name_lb_ub),
                                 x0(positions_map.at(std::get<0>(name_lb_ub))));
      }
      AddBoundingBoxConstraint(std::get<1>(name_lb_ub), std::get<2>(name_lb_ub),
                               xf(positions_map.at(std::get<0>(name_lb_ub))));
    }

    // Stitching x0 and xf (full-order model stance foot constraint)
    PrintStatus("Adding full-order model stance foot pos constraint...");
    const auto& stance_contacts = left_stance ? left_contacts : right_contacts;
    auto fom_sf_pos_constraint =
        std::make_shared<planning::FomStanceFootPosConstraint>(plant_,
                                                               stance_contacts);
    fom_sf_pos_constraint->SetConstraintScaling(
        fom_stance_ft_pos_constraint_scaling_);
    AddConstraint(fom_sf_pos_constraint, {x0.head(n_q), xf.head(n_q)});

    // Zero velocity for stance foot
    PrintStatus("Adding full-order model stance foot vel constraint...");
    auto fom_ft_vel_constraint =
        std::make_shared<planning::FomStanceFootVelConstraint>(plant_,
                                                               stance_contacts);
    fom_ft_vel_constraint->SetConstraintScaling(
        fom_stance_ft_vel_constraint_scaling_);
    if (i != 0) {
      AddConstraint(fom_ft_vel_constraint, x0);
    }
    AddConstraint(fom_ft_vel_constraint, xf);

    // Stride length constraint
    // cout << "Adding stride length constraint for full-order model...\n";
    // V1
    // AddLinearConstraint(xf(0) - x0(0) ==
    // 0.304389); V2
    /*VectorXd stride_length(1); stride_length << 0.304389 * 2;
    auto fom_sl_constraint =
    std::make_shared<planning::FomStrideLengthConstraint>( left_stance, n_q,
    stride_length); AddConstraint(fom_sl_constraint,
    {x0.head(n_q), xf.head(n_q)
                                     });*/

    // Stride length cost
    /*if (i == num_modes_ - 1) {
      cout << "Adding final position cost for full-order model...\n";
      this->AddLinearCost(-10 * xf(0));
    }*/

    counter += mode_lengths_[i] - 1;
    left_stance = !left_stance;
  }
}

void addConstraintScaling(std::unordered_map<int, double>* map,
                          vector<int> idx_vec, vector<double> s_vec) {
  DRAKE_DEMAND(idx_vec.size() == s_vec.size());
  for (int i = 0; i < idx_vec.size(); i++) {
    int idx = idx_vec[i];
    double s = s_vec[i];

    DRAKE_DEMAND(0 <= idx);
    DRAKE_DEMAND(0 < s);
    if (map->find(idx) != map->end()) {
      // Update the scaling factor
      (*map)[idx] = s;
    } else {
      // Add a new scaling factor
      map->insert(std::pair<int, double>(idx, s));
    }
  }
}

std::vector<int> CreateIdxVector(int size) {
  vector<int> ret(size);
  for (int i = 0; i < size; i++) {
    ret[i] = i;
  }
  return ret;
}

void RomTrajOpt::SetScalingForLIPM() {
  addConstraintScaling(
      &fom_discrete_dyn_constraint_scaling_, CreateIdxVector(24),
      {0.256749956352507, 0.256749956352507, 0.576854298141375,
       0.030298256032383, 0.030298256032383, 0.030298256032383,
       0.599067850424739, 0.807943702482811, 1.1232888099092,
       0.779696697984484, 0.764239696138297, 0.718478549822895,
       1.16295973251926,  1.09613666631956,  2.15622729223133,
       3.78941464911915,  9.09810486475667,  61.721918070326,
       0.368999605170422, 0.440482063570371, 0.464716406812742,
       0.365366206548824, 0.459954096581161, 0.463550985972947});
  addConstraintScaling(&fom_guard_constraint_scaling_, CreateIdxVector(4),
                       {1, 0.040500915320686, 1, 0.038541734917656});
  addConstraintScaling(&fom_stance_ft_pos_constraint_scaling_,
                       CreateIdxVector(6),
                       {0.523823492435989, 0.523823492435989, 1,
                        0.52382074853985, 0.52382074853985, 0.884415710760686});
  addConstraintScaling(
      &fom_stance_ft_vel_constraint_scaling_, CreateIdxVector(6),
      {0.28070333026431, 0.114098983149862, 0.288711940548437,
       0.254999260502145, 0.107781849536538, 0.207878166764023});
  addConstraintScaling(
      &rom_dyn_constraint_scaling_, CreateIdxVector(6),
      {0.02775672892501, 0.02775672892501, 0.027777777777778, 0.005674724775848,
       0.006428925019448, 0.027777777777778});
  addConstraintScaling(
      &rom_fom_mapping_constraint_scaling_, CreateIdxVector(6),
      {0.600254507911354, 0.600254507911354, 1, 0.277406361482681,
       0.127149946660597, 0.324725931313971});
}

void RomTrajOpt::AddTimeStepConstraint(
    std::vector<double> minimum_timestep, std::vector<double> maximum_timestep,
    bool fix_duration, bool equalize_timestep_size, double first_mode_duration,
    double remaining_mode_duration_per_mode) {
  if (fix_duration && equalize_timestep_size) {
    double dt_first_mode = first_mode_duration / (mode_lengths_[0] - 1);
    PrintStatus("Fix all timestep size in the first mode " +
                to_string(dt_first_mode));
    for (int i = 0; i < mode_lengths_[0] - 1; i++) {
      AddBoundingBoxConstraint(dt_first_mode, dt_first_mode, timestep(i));
    }
    if (num_modes_ > 1) {
      double dt_rest_of_modes =
          remaining_mode_duration_per_mode / (mode_lengths_[1] - 1);
      PrintStatus("Fix all timestep size in the rest of the modes to " +
                  to_string(dt_rest_of_modes));
      for (int i = mode_lengths_[0] - 1; i < this->N() - 1; i++) {
        AddBoundingBoxConstraint(dt_rest_of_modes, dt_rest_of_modes,
                                 timestep(i));
      }
    }
  } else {
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

    // Duration bound
    if (fix_duration) {
      double duration = first_mode_duration +
                        remaining_mode_duration_per_mode * (num_modes_ - 1);
      PrintStatus("Fix time duration: total duration = " + to_string(duration));
      AddDurationBounds(duration, duration);
    }

    // Make the timesteps between modes the same (except the first one)
    if (equalize_timestep_size) {
      PrintStatus("Equalize time steps between modes (except the first one)");
      for (int i = 2; i < num_modes_; i++) {
        if (mode_start_[i] > 0) {
          AddLinearConstraint(timestep(mode_start_[i] - 1) ==
                              timestep(mode_start_[i]));
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

void RomTrajOpt::GetStateSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const {
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());

  VectorXd times(GetSampleTimes(result));

  for (int i = 0; i < num_modes_; i++) {
    MatrixXd states_i(num_states(), mode_lengths_[i]);
    VectorXd times_i(mode_lengths_[i]);
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k_data = mode_start_[i] + j;

      VectorX<double> zk = result.GetSolution(state_vars_by_mode(i, j));

      states_i.col(j) = drake::math::DiscardGradient(zk);
      times_i(j) = times(k_data);
    }
    state_samples->push_back(states_i);
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
    const Eigen::VectorXd& x_guess_right_in_front, double w_reg_quat,
    double w_reg_xy, double w_reg_z_joints, bool straight_leg_cost) {
  PrintStatus("Adding regularization cost ...");
  int n_q = plant_.num_positions();

  // Adding cost on FOM state increases convergence rate
  // If we only add position (not velocity) in the cost, then higher cost
  // results in spacing out each step more evenly
  MatrixXd Id_quat = w_reg_quat * MatrixXd::Identity(4, 4);
  MatrixXd Id_xy = w_reg_xy * MatrixXd::Identity(2, 2);
  MatrixXd Id_z_joints = w_reg_z_joints * MatrixXd::Identity(n_q - 6, n_q - 6);

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

  bool left_stance = start_with_left_stance_;
  for (int i = 0; i < num_modes_; i++) {
    auto x_0 = x0_vars_by_mode(i);
    auto x_f = xf_vars_by_mode(i);
    if (left_stance) {
      if (i != 0) {
        fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            Id_z_joints, modifixed_x_guess_left_in_front.segment(6, n_q - 6),
            x_0.segment(6, n_q - 6)));
      }
      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_right_in_front.segment(6, n_q - 6),
          x_f.segment(6, n_q - 6)));
      //      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_left_in_front.head(4),
      //          x_0.head(4)));
      //      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_right_in_front.head(4),
      //          x_f.head(4)));
    } else {
      if (i != 0) {
        fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            Id_z_joints, modifixed_x_guess_right_in_front.segment(6, n_q - 6),
            x_0.segment(6, n_q - 6)));
      }
      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
          Id_z_joints, modifixed_x_guess_left_in_front.segment(6, n_q - 6),
          x_f.segment(6, n_q - 6)));
      //      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_right_in_front.head(4),
      //          x_0.head(4)));
      //      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
      //          Id_quat, modifixed_x_guess_left_in_front.head(4),
      //          x_f.head(4)));
    }
    if (i != 0) {
      fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
          Id_xy, final_position * i / num_modes_, x_0.segment<2>(4)));
    }
    fom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
        Id_xy, final_position * (i + 1) / num_modes_, x_f.segment<2>(4)));
    VectorX<double> quat_identity(4);
    quat_identity << 1, 0, 0, 0;
    if (i != 0) {
      fom_regularization_cost_bindings_.push_back(
          AddQuadraticErrorCost(Id_quat, quat_identity, x_0.head(4)));
    }
    fom_regularization_cost_bindings_.push_back(
        AddQuadraticErrorCost(Id_quat, quat_identity, x_f.head(4)));

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

void RomTrajOptCassie::SetHeuristicInitialGuess(
    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& r_guess,
    const Eigen::MatrixXd& dr_guess, const Eigen::MatrixXd& tau_guess,
    const Eigen::VectorXd& x_guess_left_in_front,
    const Eigen::VectorXd& x_guess_right_in_front,
    const Eigen::VectorXd& final_position, int fisrt_mode_phase_index,
    int starting_mode_index) {
  PrintStatus("Adding initial guess ...");

  MatrixXd y_guess(r_guess.rows() + dr_guess.rows(), r_guess.cols());
  y_guess << r_guess, dr_guess;

  bool left_stance = start_with_left_stance_;
  for (int i = starting_mode_index; i < num_modes_; i++) {
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
      rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
          I_h, h_guess.segment(1, 1), timestep(mode_start_[i] + j)));
    }
    // Rom states and inputs
    for (int j = 0; j < mode_lengths_[i]; j++) {
      // The intial state might start in the middle of the stride
      if (i == 0) {
        rom_regularization_cost_bindings_.push_back(
            AddQuadraticErrorCost(I_z, y_guess.col(fisrt_mode_phase_index + j),
                                  state_vars_by_mode(i, j)));
        int time_index = mode_start_[i] + j;
        rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            I_tau, tau_guess.col(fisrt_mode_phase_index + j),
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau())));
      } else {
        rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            I_z, y_guess.col(j), state_vars_by_mode(i, j)));
        int time_index = mode_start_[i] + j;
        rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            I_tau, tau_guess.col(j),
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau())));
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
