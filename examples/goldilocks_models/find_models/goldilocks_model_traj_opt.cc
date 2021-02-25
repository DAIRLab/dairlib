#include "examples/goldilocks_models/find_models/goldilocks_model_traj_opt.h"

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::trajectory_optimization::HybridDircon;

namespace dairlib {
namespace goldilocks_models {

// Constructor
GoldilocksModelTrajOpt::GoldilocksModelTrajOpt(
    const ReducedOrderModel& rom,
    std::unique_ptr<HybridDircon<double>> dircon_in,
    const MultibodyPlant<double>& plant,
    const std::vector<int>& num_time_samples,
    std::vector<DirconKinematicDataSet<double>*> constraints,
    bool is_get_nominal, const InnerLoopSetting& setting, int rom_option,
    int robot_option, double constraint_scale, bool pre_and_post_impact_efforts)
    : n_tau_(rom.n_tau()) {
  // Parameters
  bool is_add_tau_in_cost = setting.is_add_tau_in_cost;
  bool cubic_spline_in_rom_constraint =
      setting.cubic_spline_in_rom_constraint;  // for testing

  // Get total sample ponits
  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // Overlaps between modes

  // Members assignment
  dircon = std::move(dircon_in);
  num_knots_ = N;

  // Create decision variables
  // (Since VectorX allows 0-size vector, the trajectory optimization works even
  // when n_tau_ = 0.)
  tau_vars_ = dircon->NewContinuousVariables(n_tau_ * N, "tau");
  tau_post_impact_vars_ = dircon->NewContinuousVariables(
      n_tau_ * (num_time_samples.size() - 1), "tau_p");

  // Constraints
  // clang-format off
  if (!is_get_nominal) {
    // Create constraint scaling
    // TODO: re-tune this after you remove at_head and at_tail
    std::unordered_map<int, double> constraint_scale_map;
    if (rom.n_yddot() == 0) {
      // no constraint, so we don't need to scale
    } else if (robot_option == 0) {
      if (rom_option == 0) {
        //        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 /
        //        3500.0)); constraint_scale_map.insert(std::pair<int,
        //        double>(1, 1.0 / 600.0));
      } else if (rom_option == 1) {
        // Not tuned yet
      } else {
        // The scaling of others hasn't tuned yet
        DRAKE_DEMAND(false);
      }
    } else if (robot_option == 1) {
      // we can use `rom_scale` to avoid scaling the constraint down too much to
      // have effective accuracy
      double rom_scale = 100;
      // clang-format off
      if (rom_option == 0) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 3200.0 * rom_scale));
      } else if (rom_option == 1) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 3200.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(3, constraint_scale * 1.0 / 4000.0 * rom_scale));
      } else if (rom_option == 2) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 3200.0 * rom_scale));
      } else if (rom_option == 3) {
        // TODO: The scaling hasn't been tuned yet
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 3200.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 4000.0 * rom_scale));
      } else if (rom_option == 4) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 3200.0 * rom_scale));
      } else if ((rom_option == 5) || (rom_option == 6)) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 3200.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(3, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(4, constraint_scale * 1.0 / 26000.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(5, constraint_scale * 1.0 / 4000.0 * rom_scale));
      } else if (rom_option == 8) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 2600.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 2600.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 640.0 * rom_scale));
      } else {
        // The scaling of others hasn't tuned yet
        DRAKE_DEMAND(false);
      }
      // clang-format on
    } else {
      // The scaling of others hasn't tuned yet
      DRAKE_DEMAND(false);
    }
    // clang-format on

    // Create dynamics constraint (pointer) and set the scaling
    if (cubic_spline_in_rom_constraint) {
      dynamics_constraint_at_head =
          std::make_shared<find_models::DynamicsConstraint>(rom, plant, true);
      dynamics_constraint_at_tail =
          std::make_shared<find_models::DynamicsConstraint>(rom, plant, false);
      // Scaling
      dynamics_constraint_at_head->SetConstraintScaling(constraint_scale_map);
      // Not sure if scaling constraint for the tail should be the same (not
      // tuned)
      dynamics_constraint_at_tail->SetConstraintScaling(constraint_scale_map);
    } else {
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        dynamics_constraint_at_knot.push_back(
            std::make_shared<find_models::DynamicsConstraintV2>(
                rom, plant, constraints[i]));
        // Scaling
        dynamics_constraint_at_knot.at(i)->SetConstraintScaling(
            constraint_scale_map);
      }
    }

    // Impose dynamics constraints
    if (cubic_spline_in_rom_constraint) {
      // (For V1) Add dynamics constraint for all segments (between knots)
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i] - 1; j++) {
          auto x_at_knot_k = dircon->state_vars_by_mode(i, j);
          auto tau_at_knot_k = pre_and_post_impact_efforts
                                   ? tau_vars_by_mode(i, j)
                                   : reduced_model_input(N_accum + j);
          auto x_at_knot_kplus1 = dircon->state_vars_by_mode(i, j + 1);
          auto tau_at_knot_kplus1 = pre_and_post_impact_efforts
                                        ? tau_vars_by_mode(i, j + 1)
                                        : reduced_model_input(N_accum + j + 1);
          auto h_btwn_knot_k_iplus1 = dircon->timestep(N_accum + j);
          dynamics_constraint_at_head_bindings.push_back(dircon->AddConstraint(
              dynamics_constraint_at_head,
              {x_at_knot_k, tau_at_knot_k, x_at_knot_kplus1, tau_at_knot_kplus1,
               h_btwn_knot_k_iplus1}));
          if (j == num_time_samples[i] - 2) {
            // Add constraint to the end of the last segment
            dynamics_constraint_at_tail_bindings.push_back(
                dircon->AddConstraint(
                    dynamics_constraint_at_tail,
                    {x_at_knot_k, tau_at_knot_k, x_at_knot_kplus1,
                     tau_at_knot_kplus1, h_btwn_knot_k_iplus1}));
          }
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    } else {
      // (For V2) Add dynamics constraint at knot points
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i]; j++) {
          int time_index = N_accum + j;
          auto x_k = dircon->state_vars_by_mode(i, j);
          auto u_k = pre_and_post_impact_efforts
                         ? dircon->input_vars_by_mode(i, j)
                         : dircon->input(time_index);
          auto lambda_k = dircon->force(i, j);
          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          dynamics_constraint_at_knot_bindings.push_back(dircon->AddConstraint(
              dynamics_constraint_at_knot[i], {x_k, u_k, lambda_k, tau_k}));
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // variable scaling
    // TODO: need to tune variable as well.
    double tau1_scale = 26000.0;
    double tau2_scale = 4000.0;
    double tau_scale_8 = 300;
    if (robot_option == 1) {
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i]; j++) {
          int time_index = N_accum + j;
          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          if (rom_option == 1) {
            dircon->SetVariableScaling(tau_k(0), tau1_scale);
            dircon->SetVariableScaling(tau_k(1), tau2_scale);
          } else if (rom_option == 3) {
            // TODO: The scaling hasn't been tuned yet
            dircon->SetVariableScaling(tau_k(0), tau1_scale);
            dircon->SetVariableScaling(tau_k(1), tau2_scale);
          } else if ((rom_option == 5) || (rom_option == 6)) {
            // TODO: The scaling hasn't been tuned yet
            dircon->SetVariableScaling(tau_k(0), tau1_scale);
            dircon->SetVariableScaling(tau_k(1), tau1_scale);
            dircon->SetVariableScaling(tau_k(2), tau2_scale);
          } else if (rom_option == 8) {
            // TODO: The scaling hasn't been tuned yet
            dircon->SetVariableScaling(tau_k(0), tau_scale_8);
          }
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Add cost for the input tau
    double w_tau = 0;//1e-6;
    MatrixXd W = w_tau * MatrixXd::Identity(n_tau_, n_tau_);
    if (is_add_tau_in_cost) {
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i]; j++) {
          int time_index = N_accum + j;

          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          tau_cost_bindings.push_back(
              dircon->AddQuadraticCost(W, VectorXd::Zero(n_tau_), tau_k));
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Add constraint on input tau
    if (rom_option == 8) {
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i]; j++) {
          int time_index = N_accum + j;
          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          dircon->AddBoundingBoxConstraint(
              0, std::numeric_limits<double>::infinity(), tau_k(0));
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    }
  }

}  // end of constructor

Eigen::VectorBlock<const VectorXDecisionVariable>
GoldilocksModelTrajOpt::reduced_model_input(int index) const {
  DRAKE_DEMAND(index >= 0 && index < num_knots_);
  return tau_vars_.segment(index * n_tau_, n_tau_);
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// GoldilocksModelTrajOpt::reduced_model_position(int index, int n_s) const {
//   DRAKE_DEMAND(index >= 0 && index < num_knots_);
//   return s_vars_.segment(index * n_s, n_s);
// }

const Eigen::VectorBlock<const VectorXDecisionVariable>
GoldilocksModelTrajOpt::tau_post_impact_vars_by_mode(int mode) const {
  return tau_post_impact_vars_.segment(mode * n_tau_, n_tau_);
}

VectorXDecisionVariable GoldilocksModelTrajOpt::tau_vars_by_mode(
    int mode, int time_index) const {
  if (time_index == 0 && mode > 0) {
    return tau_post_impact_vars_by_mode(mode - 1);
  } else {
    return tau_vars_.segment((dircon->mode_start()[mode] + time_index) * n_tau_,
                             n_tau_);
  }
}

// (This is modified from HybridDircon::ReconstructStateTrajectory)
// Instead of returning a trajectory class, we
// create time, state and its derivatives required for reconstructing the cubic
// spline
void GoldilocksModelTrajOpt::ConstructStateCubicSplineInfo(
    const MathematicalProgramResult& result,
    const MultibodyPlant<double>& plant,
    const std::vector<int>& num_time_samples,
    vector<DirconKinematicDataSet<double>*> constraints, Eigen::VectorXd* times,
    Eigen::MatrixXd* states, Eigen::MatrixXd* derivatives) const {
  int num_modes = num_time_samples.size();

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // because of overlaps between modes

  VectorXd times_all(dircon->GetSampleTimes(result));
  times->resize(N + num_modes - 1);

  states->resize(plant.num_positions() + plant.num_velocities(),
                 N + num_modes - 1);
  derivatives->resize(plant.num_positions() + plant.num_velocities(),
                      N + num_modes - 1);
  MatrixXd inputs(plant.num_actuators(), N + num_modes - 1);

  int mode_start = 0;
  for (int i = 0; i < num_modes; i++) {
    for (int j = 0; j < num_time_samples[i]; j++) {
      int k = mode_start + j + i;
      int k_data = mode_start + j;
      (*times)(k) = times_all(k_data);

      // False timestep to match velocities
      if (i > 0 && j == 0) {
        (*times)(k) += +1e-6;
      }
      VectorX<double> xk = result.GetSolution(dircon->state_vars_by_mode(i, j));
      VectorX<double> uk = result.GetSolution(dircon->input(k_data));
      states->col(k) = xk;
      inputs.col(k) = uk;
      auto context = multibody::createContext<double>(plant, xk, uk);
      constraints[i]->updateData(*context,
                                 result.GetSolution(dircon->force(i, j)));
      derivatives->col(k) =
          drake::math::DiscardGradient(constraints[i]->getXDot());
    }
    mode_start += num_time_samples[i] - 1;
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
