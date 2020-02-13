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
    vector<double> var_scale,
    int robot_option):
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
  double tau_scale = var_scale[5];

  // Add cost for the input tau
  if (is_add_tau_in_cost) {
    for (int i = 0; i < N; i++) {
      auto tau_i = reduced_model_input(i, n_tau);
      tau_cost_bindings.push_back(dircon->AddQuadraticCost(
                                    tau_scale * tau_scale * MatrixXd::Identity(n_tau, n_tau),
                                    VectorXd::Zero(n_tau),
                                    tau_i));
    }
  }

  // Constraints
  if (!is_get_nominal) {
    // Create dynamics constraint (pointer)
    dynamics_constraint_at_head = make_shared<find_models::DynamicsConstraint>(
                                    n_s, n_feature_s, theta_s,
                                    n_sDDot, n_feature_sDDot, theta_sDDot,
                                    n_tau, B_tau, plant, plant_double,
                                    var_scale, tau_scale, true,
                                    robot_option);
    // dynamics_constraint_at_tail = make_shared<find_models::DynamicsConstraint>(
    //                                n_s, n_feature_s, theta_s,
    //                                n_sDDot, n_feature_sDDot, theta_sDDot,
    //                                n_tau, B_tau, plant, plant_double,
    //                                var_scale, tau_scale, false,
    //                                robot_option);

    // Add dynamics constraint for all segments (between knots)
    int N_accum = 0;
    for (unsigned int i = 0; i < num_time_samples.size() ; i++) {
      // cout << "i = " << i << endl;
      // cout << "N_accum = " << N_accum << endl;
      for (int j = 0; j < num_time_samples[i] - 1 ; j++) {
        // cout << "    j = " << j << endl;
        auto x_at_knot_k = dircon->state_vars_by_mode(i, j);
        auto tau_at_knot_k = reduced_model_input(N_accum + j, n_tau);
        auto x_at_knot_kplus1 = dircon->state_vars_by_mode(i, j + 1);
        auto tau_at_knot_kplus1 = reduced_model_input(N_accum + j + 1, n_tau);
        auto h_btwn_knot_k_iplus1 = dircon->timestep(N_accum + j);
        dynamics_constraint_at_head_bindings.push_back(dircon->AddConstraint(
              dynamics_constraint_at_head, {x_at_knot_k, tau_at_knot_k,
                                            x_at_knot_kplus1, tau_at_knot_kplus1, h_btwn_knot_k_iplus1
                                           }));
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


}  // namespace goldilocks_models
}  // namespace dairlib

