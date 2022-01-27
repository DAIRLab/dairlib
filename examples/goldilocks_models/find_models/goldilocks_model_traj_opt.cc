#include "examples/goldilocks_models/find_models/goldilocks_model_traj_opt.h"

#include "lcm/rom_planner_saved_trajectory.h"

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
    const drake::multibody::MultibodyPlant<double>& plant,
    std::vector<int> num_time_samples, std::vector<double> minimum_timestep,
    std::vector<double> maximum_timestep,
    std::vector<DirconKinematicDataSet<double>*> constraints,
    std::vector<DirconOptions> options, const ReducedOrderModel& rom,
    const ReducedOrderModel& mirrored_rom, bool is_get_nominal,
    const InnerLoopSetting& setting, int rom_option, int robot_option,
    double constraint_scale, bool pre_and_post_impact_efforts)
    : HybridDircon<double>(plant, num_time_samples, minimum_timestep,
                           maximum_timestep, constraints, options,
                           pre_and_post_impact_efforts),
      n_tau_(rom.n_tau()) {
  // Parameters
  bool is_add_tau_in_cost = setting.is_add_tau_in_cost;
  bool cubic_spline_in_rom_constraint =
      setting.cubic_spline_in_rom_constraint;  // for testing

  // Get total sample ponits
  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // Overlaps between modes

  // Members assignment
  num_knots_ = N;

  // Create decision variables
  // (Since VectorX allows 0-size vector, the trajectory optimization works even
  // when n_tau_ = 0.)
  tau_vars_ = NewContinuousVariables(n_tau_ * N, "tau");
  tau_post_impact_vars_ =
      NewContinuousVariables(n_tau_ * (num_time_samples.size() - 1), "tau_p");

  // Testing -- put a ROM MPC planned traj back into FOM trajopt via constraint
  bool testing_mpc_planned_traj = false;
  if (testing_mpc_planned_traj) {
    double duration = 0.35;

    bool include_rom_vel = true;

    // Read in a MPC planned trajectory
    string path = "";
    path =
        "/home/yuming/workspace/dairlib_data/goldilocks_models/planning/"
        "robot_1/data/0_rom_trajectory";
    auto rom_traj = ReadRomPlannerTrajectory(path, true);
    cout << "rom_traj.get_segment_times() = \n";
    for (auto time : rom_traj.get_segment_times()) {
      cout << time << endl;
    }
    // Align 0 to the start of the second mode
    //    rom_traj.shiftRight(-rom_traj.get_segment_times()[1]);
    //    cout << "rom_traj.start_time() = " << rom_traj.start_time();
    //    cout << "rom_traj.end_time() = " << rom_traj.end_time();

    // Constant Kinematics constraint
    vector<int> active_dim = {0, 1, 2};
    for (int i = 0; i < N; i++) {
      //      if (i == 1) active_dim = {2};

      double t = duration * i / (N - 1);
      if (t == duration)
        t -= 1e-8;  // PP evaluates from the right at discontinuity
      VectorXd const_value = rom_traj.value(t);
      if (!include_rom_vel) const_value.conservativeResize(rom.n_y());
      cosnt_kinematics_constraint.push_back(
          std::make_shared<ConstKinematicsConstraint>(
              rom, plant, const_value, include_rom_vel, active_dim));
      //      cout << const_value.transpose() << endl;
      cout << rom_traj.value(t).transpose() << endl;
    }

    // vector<int> j_knot = {0};
    // vector<int> j_knot = {1, 19};
    // vector<int> j_knot = {0, 6, 12, 19};
    // vector<int> j_knot = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 19};
    vector<int> j_knot = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
    for (int j : j_knot) {
      //      for (int j = 0; j < num_time_samples[0]; j=j+9) {
      cout << "impose ROM state constraint on knot j = " << j << endl;
      auto x_k = state_vars_by_mode(0, j);
      cosnt_kinematics_constraint_bindings.push_back(AddConstraint(
          cosnt_kinematics_constraint[j],
          include_rom_vel ? x_k : x_k.head(plant.num_positions())));
    }
  }

  // Constraints
  if (!is_get_nominal) {
    //  if (!is_get_nominal && !testing_mpc_planned_traj) {
    //    if (testing_mpc_planned_traj) cout << "Imposing regular ROM
    //    constraints\n";

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
      double rom_scale = 1;
      // clang-format off
      if (rom_option == 0) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 32.0 * rom_scale));
      } else if (rom_option == 1) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 32.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(3, constraint_scale * 1.0 / 40.0 * rom_scale));
      } else if (rom_option == 2) {
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 32.0 * rom_scale));
      } else if (rom_option == 3) {
        // TODO: The scaling hasn't been tuned yet
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 32.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 40.0 * rom_scale));
      } else if (rom_option == 4) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 32.0 * rom_scale));
      } else if ((rom_option == 5) || (rom_option == 6)) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 32.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(3, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(4, constraint_scale * 1.0 / 260.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(5, constraint_scale * 1.0 / 40.0 * rom_scale));
      } else if ((rom_option >= 8) && (rom_option <= 16)) {
        // TODO: The scaling hasn't been tuned yet. These are just guessings
        constraint_scale_map.insert(std::pair<int, double>(0, constraint_scale * 1.0 / 26.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(1, constraint_scale * 1.0 / 26.0 * rom_scale));
        constraint_scale_map.insert(std::pair<int, double>(2, constraint_scale * 1.0 / 6.4 * rom_scale));
      } else {
        // The scaling of others hasn't tuned yet
        DRAKE_DEMAND(false);
      }
      // clang-format on
    } else {
      // The scaling of others hasn't tuned yet
      DRAKE_DEMAND(false);
    }

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
      bool use_mirrored = false;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        dynamics_constraint_at_knot.push_back(
            std::make_shared<find_models::DynamicsConstraintV2>(
                use_mirrored ? mirrored_rom : rom, plant, constraints[i]));
        // Scaling
        dynamics_constraint_at_knot.at(i)->SetConstraintScaling(
            constraint_scale_map);
        use_mirrored = !use_mirrored;
      }
    }

    // Impose dynamics constraints
    if (cubic_spline_in_rom_constraint) {
      // (For V1) Add dynamics constraint for all segments (between knots)
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i] - 1; j++) {
          auto x_at_knot_k = state_vars_by_mode(i, j);
          auto tau_at_knot_k = pre_and_post_impact_efforts
                                   ? tau_vars_by_mode(i, j)
                                   : reduced_model_input(N_accum + j);
          auto x_at_knot_kplus1 = state_vars_by_mode(i, j + 1);
          auto tau_at_knot_kplus1 = pre_and_post_impact_efforts
                                        ? tau_vars_by_mode(i, j + 1)
                                        : reduced_model_input(N_accum + j + 1);
          auto h_btwn_knot_k_iplus1 = timestep(N_accum + j);
          dynamics_constraint_at_head_bindings.push_back(
              AddConstraint(dynamics_constraint_at_head,
                            {x_at_knot_k, tau_at_knot_k, x_at_knot_kplus1,
                             tau_at_knot_kplus1, h_btwn_knot_k_iplus1}));
          if (j == num_time_samples[i] - 2) {
            // Add constraint to the end of the last segment
            dynamics_constraint_at_tail_bindings.push_back(
                AddConstraint(dynamics_constraint_at_tail,
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
          // if (testing_mpc_planned_traj && (i == 0) && (j == 0)) continue;
          int time_index = N_accum + j;
          auto x_k = state_vars_by_mode(i, j);
          auto u_k = pre_and_post_impact_efforts ? input_vars_by_mode(i, j)
                                                 : input(time_index);
          auto lambda_k = force(i, j);
          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          dynamics_constraint_at_knot_bindings.push_back(AddConstraint(
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
            SetVariableScaling(tau_k(0), tau1_scale);
            SetVariableScaling(tau_k(1), tau2_scale);
          } else if (rom_option == 3) {
            // TODO: The scaling hasn't been tuned yet
            SetVariableScaling(tau_k(0), tau1_scale);
            SetVariableScaling(tau_k(1), tau2_scale);
          } else if ((rom_option == 5) || (rom_option == 6)) {
            // TODO: The scaling hasn't been tuned yet
            SetVariableScaling(tau_k(0), tau1_scale);
            SetVariableScaling(tau_k(1), tau1_scale);
            SetVariableScaling(tau_k(2), tau2_scale);
          } else if (rom_option == 8) {
            // TODO: The scaling hasn't been tuned yet
            SetVariableScaling(tau_k(0), tau_scale_8);
          }
        }
        N_accum += num_time_samples[i];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Add cost for the input tau
    double w_tau = 0;  // 1e-6;
    if (is_add_tau_in_cost && (n_tau_ > 0) && (w_tau > 0)) {
      MatrixXd W = w_tau * MatrixXd::Identity(n_tau_, n_tau_);
      int N_accum = 0;
      for (unsigned int i = 0; i < num_time_samples.size(); i++) {
        for (int j = 0; j < num_time_samples[i]; j++) {
          int time_index = N_accum + j;

          auto tau_k = pre_and_post_impact_efforts
                           ? tau_vars_by_mode(i, j)
                           : reduced_model_input(time_index);
          cost_tau_bindings_.push_back(
              AddQuadraticCost(W, VectorXd::Zero(n_tau_), tau_k));
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
          AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                                   tau_k(0));
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
    return tau_vars_.segment((mode_start()[mode] + time_index) * n_tau_,
                             n_tau_);
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
