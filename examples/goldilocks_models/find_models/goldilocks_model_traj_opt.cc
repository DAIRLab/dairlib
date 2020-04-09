#include "examples/goldilocks_models/find_models/goldilocks_model_traj_opt.h"


namespace dairlib {
namespace goldilocks_models {

// Constructor
GoldilocksModelTrajOpt::GoldilocksModelTrajOpt(int n_s, int n_sDDot, int n_tau,
    int n_feature_s, int n_feature_sDDot,
    MatrixXd B_tau, const VectorXd & theta_s, const VectorXd & theta_sDDot,
    std::unique_ptr<HybridDircon<double>> dircon_in,
    const MultibodyPlant<AutoDiffXd> * plant,
    const MultibodyPlant<double> * plant_double,
    const std::vector<int> & num_time_samples,
    bool is_get_nominal,
    bool is_add_tau_in_cost,
    int rom_option, int robot_option):
  n_s_(n_s),
  n_sDDot_(n_sDDot),
  n_tau_(n_tau),
  n_feature_s_(n_feature_s),
  n_feature_sDDot_(n_feature_sDDot) {

  // Get total sample ponits
  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++)
    N += num_time_samples[i];
  N -= num_time_samples.size() - 1; //Overlaps between modes

  // Members assignment
  dircon = std::move(dircon_in);
  num_knots_ = N;

  // Create decision variables
  // (Since VectorX allows 0-size vector, the trajectory optimization works even
  // when n_tau = 0.)
  tau_vars_ = dircon->NewContinuousVariables(n_tau * N, "tau");

  // Constraints
  if (!is_get_nominal) {
    // Create dynamics constraint (pointer)
    dynamics_constraint_at_head = make_shared<find_models::DynamicsConstraint>(
                                    n_s, n_feature_s, theta_s,
                                    n_sDDot, n_feature_sDDot, theta_sDDot,
                                    n_tau, B_tau, plant, plant_double,
                                    true, rom_option,
                                    robot_option);
    // dynamics_constraint_at_tail = make_shared<find_models::DynamicsConstraint>(
    //                                n_s, n_feature_s, theta_s,
    //                                n_sDDot, n_feature_sDDot, theta_sDDot,
    //                                n_tau, B_tau, plant, plant_double,
    //                                false, rom_option,
    //                                robot_option);

    // Constraint scaling
    // TODO: re-tune this after you remove at_head and at_tail
    std::unordered_map<int, double> constraint_scale_map;
    if (n_sDDot == 0) {
      // no constraint, so we don't need to scale
    } else if (robot_option == 0) {
      if (rom_option == 0) {
//        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 / 3500.0));
//        constraint_scale_map.insert(std::pair<int, double>(1, 1.0 / 600.0));
      } else {
        // The scaling of others hasn't tuned yet
        DRAKE_DEMAND(false);
      }
    } else if (robot_option == 1) {
      if (rom_option == 0) {
        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 / 26000.0));
        constraint_scale_map.insert(std::pair<int, double>(1, 1.0 / 3200.0));
      } else if (rom_option == 1) {
        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 / 26000.0));
        constraint_scale_map.insert(std::pair<int, double>(1, 1.0 / 3200.0));
        constraint_scale_map.insert(std::pair<int, double>(2, 1.0 / 26000.0));
        constraint_scale_map.insert(std::pair<int, double>(3, 1.0 / 4000.0));
      } else if (rom_option == 2) {
        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 / 3200.0));
      } else if (rom_option == 3) {
        // TODO: The scaling hasn't been tuned yet
        constraint_scale_map.insert(std::pair<int, double>(0, 1.0 / 3200.0));
        constraint_scale_map.insert(std::pair<int, double>(1, 1.0 / 26000.0));
        constraint_scale_map.insert(std::pair<int, double>(2, 1.0 / 4000.0));
      } else {
        // The scaling of others hasn't tuned yet
        DRAKE_DEMAND(false);
      }
    } else {
      // The scaling of others hasn't tuned yet
      DRAKE_DEMAND(false);
    }
    dynamics_constraint_at_head->SetConstraintScaling(constraint_scale_map);

    // variable scaling
    // TODO: need to tune variable as well.
    double tau1_scale = 26000.0;
    double tau2_scale = 4000.0;
    if (robot_option == 1) {
      if (rom_option == 1) {
        for (int i = 0; i < N; i++) {
          auto tau_i = reduced_model_input(i, n_tau);
          dircon->SetVariableScaling(tau_i(0), tau1_scale);
          dircon->SetVariableScaling(tau_i(1), tau2_scale);
        }
      } else if (rom_option == 3) {
        for (int i = 0; i < N; i++) {
          auto tau_i = reduced_model_input(i, n_tau);
          // TODO: The scaling hasn't been tuned yet
          dircon->SetVariableScaling(tau_i(0), tau1_scale);
          dircon->SetVariableScaling(tau_i(1), tau2_scale);
        }
      }
    }

    // Add cost for the input tau
    double w_tau = 1e-6;
    if (is_add_tau_in_cost) {
      for (int i = 0; i < N; i++) {
        MatrixXd W = w_tau * MatrixXd::Identity(n_tau, n_tau);

        if (robot_option == 1) {
          if (rom_option == 1) {
            W(0, 0) /= (tau1_scale * tau1_scale);
            W(1, 1) /= (tau2_scale * tau2_scale);
          } else if (rom_option == 3) {
            // TODO: hasn't added
            W(0, 0) /= (tau1_scale * tau1_scale);
            W(1, 1) /= (tau2_scale * tau2_scale);
          }
        }

        auto tau_i = reduced_model_input(i, n_tau);
        tau_cost_bindings.push_back(
            dircon->AddQuadraticCost(W, VectorXd::Zero(n_tau), tau_i));
      }
    }

    // Add dynamics constraint for all segments (between knots)
    int N_accum = 0;
    for (unsigned int i = 0; i < num_time_samples.size(); i++) {
      // cout << "i = " << i << endl;
      // cout << "N_accum = " << N_accum << endl;
      for (int j = 0; j < num_time_samples[i] - 1; j++) {
        // cout << "    j = " << j << endl;
        auto x_at_knot_k = dircon->state_vars_by_mode(i, j);
        auto tau_at_knot_k = reduced_model_input(N_accum + j, n_tau);
        auto x_at_knot_kplus1 = dircon->state_vars_by_mode(i, j + 1);
        auto tau_at_knot_kplus1 = reduced_model_input(N_accum + j + 1, n_tau);
        auto h_btwn_knot_k_iplus1 = dircon->timestep(N_accum + j);
        dynamics_constraint_at_head_bindings.push_back(
            dircon->AddConstraint(dynamics_constraint_at_head,
                                  {x_at_knot_k, tau_at_knot_k, x_at_knot_kplus1,
                                   tau_at_knot_kplus1, h_btwn_knot_k_iplus1}));
        // dynamics_constraint_at_tail_bindings.push_back(dircon->AddConstraint(
        //   dynamics_constraint_at_tail, {x_at_knot_k, tau_at_knot_k,
        //     x_at_knot_kplus1, tau_at_knot_kplus1, h_btwn_knot_k_iplus1}));
      }
      N_accum += num_time_samples[i];
      N_accum -= 1;  // due to overlaps between modes
    }
  }

}  // end of constructor


Eigen::VectorBlock<const VectorXDecisionVariable>
GoldilocksModelTrajOpt::reduced_model_input(int index, int n_tau) const {
  DRAKE_DEMAND(index >= 0 && index < num_knots_);
  return tau_vars_.segment(index * n_tau, n_tau);
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// GoldilocksModelTrajOpt::reduced_model_position(int index, int n_s) const {
//   DRAKE_DEMAND(index >= 0 && index < num_knots_);
//   return s_vars_.segment(index * n_s, n_s);
// }

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
      auto context = multibody::createContext(plant, xk, uk);
      constraints[i]->updateData(*context, result.GetSolution(dircon->force(i, j)));
      derivatives->col(k) =
          drake::math::DiscardGradient(constraints[i]->getXDot());
    }
    mode_start += num_time_samples[i] -1;
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib

