#include "ic3_tracking_controller.h"
#include <Eigen/Dense>

#include <c3/core/c3_miqp.h>
#include <c3/core/c3_qp.h>
#include <c3/core/c3_plus.h>
#include "c3/systems/common/quaternion_error_hessian.h"
#include <iostream>

#include <c3/multibody/geom_geom_collider.h>
#include <c3/core/lcs.h>

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using c3::C3;
using c3::C3MIQP;
using c3::C3QP;
using c3::C3Plus;
using c3::LCS;
using c3::LCSFactory;
using c3::C3Options;
using c3::systems::C3ControllerOptions;
using c3::systems::C3Output;
using c3::LCSFactoryOptions;
using c3::ContactPairConfig;

using std::vector;
using systems::TimestampedVector;

namespace systems {

iC3TrackingController::iC3TrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, C3ControllerOptions controller_options, 
      iC3Options ic3_options, int example_idx, 
      MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u, 
      drake::systems::Context<double>& plant_context, vector<SortedPair<GeometryId>> contact_geoms)
    : plant_(plant),
      controller_options_(controller_options),
      c3_options_(controller_options.c3_options),
      lcs_factory_options_(controller_options.lcs_factory_options),
      ic3_options_(ic3_options),
      N_(lcs_factory_options_.N), 
      dt_(lcs_factory_options_.dt),
      dt_scaling_(lcs_factory_options_.dt / ic3_options_.dt),
      example_idx_(example_idx),
      A_x_(A_x),
      lb_x_(lb_x),
      ub_x_(ub_x), 
      A_u_(A_u),
      lb_u_(lb_u),
      ub_u_(ub_u),
      contact_geoms_(contact_geoms),
      plant_context_(plant_context) { // Only used for debugging
  this->set_name("ic3_tracking_controller");


  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q * dt_scaling_);
    R_.push_back(discount_factor * c3_options_.R * dt_scaling_);
    G_.push_back(discount_factor * c3_options_.G * dt_scaling_);
    U_.push_back(discount_factor * c3_options_.U * dt_scaling_);

    discount_factor *= c3_options_.gamma;
  }
  Q_.push_back(discount_factor * c3_options_.Q * dt_scaling_);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);
  DRAKE_DEMAND(G_.size() == N_);
  DRAKE_DEMAND(U_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;


  solve_time_filter_constant_ = controller_options_.solve_time_filter_alpha;
  
  int n_lambda_with_tangential = 0;
  int num_contacts = lcs_factory_options_.num_contacts;
  for (ContactPairConfig pair : lcs_factory_options_.contact_pair_configs.value()) {
    int num_pairs = pair.body_A_collision_geom_indices.size() * pair.body_B_collision_geom_indices.size();
    n_lambda_with_tangential += 2 * num_pairs * pair.num_friction_directions;
  }

  if (lcs_factory_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * num_contacts + n_lambda_with_tangential;
  } else if (lcs_factory_options_.contact_model == "anitescu") {
    n_lambda_ = n_lambda_with_tangential;
  } else {
    std::cerr << ("Unknown or unsupported contact model: " +
      lcs_factory_options_.contact_model) << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  VectorXd zeros = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);
  n_u_ = plant_.num_actuators();

  // Get quaternion bodies
  for (const auto& body_idx : plant_.GetFloatingBaseBodies()) {
    const auto& body = plant_.get_body(body_idx);
    int start = body.floating_positions_start();
    quaternion_indices_.push_back(start);
  }

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  if (controller_options_.projection_type == "MIQP") {
    c3_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                   C3::CostMatrices(Q_, R_, G_, U_),
                                   x_desired_placeholder, c3_options_);

  } else if (controller_options_.projection_type == "QP") {
    c3_ = std::make_unique<C3QP>(lcs_placeholder,
                                 C3::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else if (controller_options_.projection_type == "C3+") {
    c3_ = std::make_unique<C3Plus>(lcs_placeholder,
                                 C3::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  // c3_->SetOsqpSolverOptions(solver_options_);
  tracking_target_ = VectorXd::Zero(n_u_);

  // HARDCODED
  int ee_start_idx;
  int ee_size = ic3_options_.ee_tracking_vector.size();
  if (example_idx_ == 0) {
    ee_start_idx = 0;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    ee_start_idx = 0;
  }
  c3_->AddEETrackingCost(ee_start_idx, ee_size);

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  lcs_input_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs_placeholder))
          .get_index();

  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();

  // Stuff specific to tracking ic3
  lqr_input_port_ =
      this->DeclareAbstractInputPort("lqr_input", drake::Value<lcmt_lqr_output>())
          .get_index();

  ic3_x_port_ =
      this->DeclareAbstractInputPort("ic3_x_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  ic3_u_port_ =
      this->DeclareAbstractInputPort("ic3_u_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  ic3_lambda_port_ =
      this->DeclareAbstractInputPort("ic3_lambda_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  timestep_port_ =  
      this->DeclareVectorInputPort("timestep_port", 1).get_index();
          
  auto c3_solution = c3::systems::C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  c3_solution_port_ =
      this->DeclareAbstractOutputPort("c3_solution", c3_solution,
                                      &iC3TrackingController::OutputC3Solution)
          .get_index();

  tracking_target_port_ =
      this->DeclareVectorOutputPort(
              "tracking_target_port",
							BasicVector<double>(n_u_),
              &iC3TrackingController::OutputTrackingTarget)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_index_ = DeclareDiscreteState(n_x_);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

  if (controller_options_.publish_frequency > 0) {
    DeclarePeriodicDiscreteUpdateEvent(1 / controller_options_.publish_frequency, 0.0,
                                       &iC3TrackingController::ComputePlan);
  } else {
    DeclareForcedDiscreteUpdateEvent(&iC3TrackingController::ComputePlan);
  }

}

LCS iC3TrackingController::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, N_, dt_);
}

drake::systems::EventStatus iC3TrackingController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const BasicVector<double>& timestep_vector =
      *this->template EvalVectorInput<BasicVector>(context, timestep_port_);
  int ic3_timestep = static_cast<int>(timestep_vector.get_value()(0));

  std::cout << "context time " << context.get_time() << std::endl;

  // If in teleop or waiting, don't solve
  if (ic3_timestep < 0) return drake::systems::EventStatus::Succeeded();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "ic3 timestep " << ic3_timestep << std::endl;

  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  auto& lcs =
      this->EvalAbstractInput(context, lcs_input_port_)->get_value<LCS>();
  drake::VectorX<double> x_lcs = lcs_x->get_data();

  auto& x_pred = context.get_discrete_state(x_pred_index_).value();
  auto mutable_x_pred = discrete_state->get_mutable_value(x_pred_index_);
  auto mutable_solve_time =
      discrete_state->get_mutable_value(filtered_solve_time_index_);

  const std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);
  const auto& lcm_all_x_trajectories = 
    this->EvalAbstractInput(context, ic3_x_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_u_trajectories = 
    this->EvalAbstractInput(context, ic3_u_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_lambda_trajectories = 
    this->EvalAbstractInput(context, ic3_lambda_port_)->get_value<lcmt_timestamped_saved_traj>();
  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);
  LcmTrajectory lambda_trajectory = LcmTrajectory(lcm_all_lambda_trajectories.saved_traj);
  LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory input_trajectory = u_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory force_trajectory = lambda_trajectory.GetTrajectory(trajectory_name);
  MatrixXd state_data = state_trajectory.datapoints;
  MatrixXd input_data = input_trajectory.datapoints;
  MatrixXd force_data = force_trajectory.datapoints;


  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();

  // Assumes 1 quaternion body
  vector<VectorXd> x_desired;
  x_desired.push_back(x_des.value());
  VectorXd x_curr = x_lcs;
  for (int i = 0; i < N_; i++) {
    int idx = std::min(ic3_options_.N-1, ic3_timestep+i);
    VectorXd u_nominal = input_data.col(idx);
    x_curr = lcs.Simulate(x_curr, u_nominal);
    double norm = x_curr.norm(); // slightly hacky correction term
    std::cout << "norm " << i << " " << norm << std::endl;

    VectorXd x_des_copy = x_des.value();
    x_des_copy.segment(quaternion_indices_[0], 4) = 
        x_des_copy.segment(quaternion_indices_[0], 4) * norm;
    x_desired.push_back(x_des_copy);
  }


  // Penetration testing code
  // if (n_u_ == 9) {
  //   VectorXd x_plan = state_data.col(ic3_timestep);
  //   c3::LCSSimulateConfig config; 
  //   config.regularized = true;
  //   config.max_exp = -6;
    
  //   VectorXd u_test(VectorXd::Zero(n_u_));
  //   u_test(2) = 0.196;
  //   u_test(5) = 0.196;
  //   u_test(8) = 0.196;
  //   auto [x_out, force] = lcs.SimulateAndReturnForce(x_plan, u_test, config);

  //   for (int f = 0; f < 3; f++) {
  //     plant_.SetPositionsAndVelocities(&plant_context_, x_plan);
  //     c3::multibody::GeomGeomCollider collider(plant_, contact_geoms_[f]);
  //     auto [phi, J] = collider.EvalPolytope(plant_context_, 2);
  //     std::cout << "phi " << f << " " << phi << std::endl;
  //     std::cout << "simulated lambda " << force.segment(4*f, 4).transpose() << std::endl << std::endl;
  //   }
  // }


  UpdateQuaternionCosts(x_lcs, x_des.value());
  C3::CostMatrices new_costs(Q_, R_, G_, U_);

  c3_->UpdateCostMatrices(new_costs);
  c3_->UpdateTarget(x_desired);

  int value_function_idx;
  vector<Eigen::MatrixXd> H;
  vector<Eigen::VectorXd> g;

  // Update final cost with LQR value function
  const auto* lqr_input = this->EvalAbstractInput(context, lqr_input_port_);
  if (lqr_input != nullptr && 0 <= ic3_timestep && 
        ic3_timestep <= ic3_options_.N - N_ * dt_scaling_) {
    const auto& lqr_all_inputs = lqr_input->get_value<lcmt_lqr_output>();

    const std::vector<std::vector<std::vector<double>>>& source_H = lqr_all_inputs.H[ic3_options_.iter_to_use];
    const std::vector<std::vector<double>>& source_g = lqr_all_inputs.g[ic3_options_.iter_to_use];

    for (int i = 0; i < source_H.size(); i++) {

      Eigen::MatrixXd mat(MatrixXd::Zero(source_H[i].size(), source_H[i][0].size()));
      for (int j = 0; j < source_H[i].size(); j++) {
          mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_H[i][j].data(), source_H[i][j].size());
      }
      H.push_back(std::move(mat));
      g.push_back(Eigen::VectorXd::Map(source_g[i].data(), source_g[i].size()));
    }

    int lower_idx = std::max(0, ic3_timestep - ic3_options_.value_function_search_size);
    int upper_idx = std::min(ic3_options_.N, ic3_timestep + ic3_options_.value_function_search_size);

    // std::cout << "l idx " << lower_idx << " u idx " << upper_idx << std::endl;

    int best_idx = GetNearestXForValueFunction(x_lcs, state_data.middleCols(lower_idx, upper_idx-lower_idx+1)) + lower_idx;
    value_function_idx = std::min(ic3_options_.N, best_idx + static_cast<int>(N_ * dt_scaling_)); // Want H[k + N]
    c3_->UpdateFinalCost(H[value_function_idx], g[value_function_idx]);


    if (ic3_options_.add_acceleration_cost) {
      c3_->AddAccelerationCost(n_q_, n_v_, ic3_options_.acceleration_cost_weight);
    }

    c3_->UpdateLambdaMatchingCost(ic3_options_.lambda_tracking_weight * MatrixXd::Identity(n_lambda_, n_lambda_));
    c3_->UpdateLambdaMatchingTarget(force_data.col(value_function_idx));

    // Set x_des of ee to match ic3 plan
    if (0 <= ic3_timestep) {
      int ee_start_idx;
      int ee_size;

      // HARDCODED
      if (example_idx_ == 0) {
        ee_start_idx = 0;
        ee_size = 5;
      } else if (example_idx_ == 1 || example_idx_ == 2) {
        ee_start_idx = 0;
        ee_size = 9;
      }

      vector<VectorXd> ee_x_des;
      for (int i = 0; i < N_+1; i++) {
        int idx = std::min(static_cast<int>(best_idx + i * dt_scaling_), ic3_options_.N); 

        if (n_x_ == 23) {
          ee_x_des.push_back(state_data.col(idx).segment(ee_start_idx, ee_size));
        } else if (n_x_ == 31) {
          // HARDCODED indices, compute rigid body transform between cube and ee in plan
          VectorXd x_plan = state_data.col(idx);
          VectorXd ee_x_des_curr(ee_size);
            
          Eigen::Quaterniond q_world_body(x_plan(9), x_plan(10), x_plan(11), x_plan(12)); 
          Eigen::Vector3d p_world_body(x_plan(13), x_plan(14), x_plan(15));    

          for (int f = 0; f < 3; f++) {
            Eigen::Vector3d p_world_point(x_plan(3*f), x_plan(3*f+1), x_plan(3*f+2));   

            // Get relative transform of finger in cube frame
            drake::math::RigidTransform<double> X_WB(drake::math::RotationMatrix<double>(q_world_body), p_world_body);
            drake::math::RigidTransform<double> X_BW = X_WB.inverse();
            Eigen::Vector3d p_body_point = X_BW * p_world_point;
                  
            // Get corresponding point in world for current cube pose
            Eigen::Quaterniond q_world_body_curr(x_lcs(9), x_lcs(10), x_lcs(11), x_lcs(12));
            Eigen::Vector3d p_world_body_curr(x_lcs(13), x_lcs(14), x_lcs(15));
            drake::math::RigidTransform<double> X_WB_curr(
                drake::math::RotationMatrix<double>(q_world_body_curr), 
                p_world_body_curr
            );
            ee_x_des_curr.segment(3*f, 3) = X_WB_curr * p_body_point;

            // HARDCODED OFFSET 
            if (example_idx_ == 1){
              ee_x_des_curr(3*f+2) = 0.07;
            }
          }
          ee_x_des.push_back(ee_x_des_curr);
        }
      }

      MatrixXd Q_ee = ic3_options_.ee_tracking_weight * ic3_options_.ee_tracking_vector.asDiagonal();
      c3_->UpdateEETrackingTargetAndCost(ee_x_des, Q_ee);

      // Add constraint to stay near ee of plan
      if (example_idx_ == 1 && ic3_options_.add_constraints_follow_plan) {
        for (int f = 0; f < 3; f++) {
          // A_x_(3*f, 3*f) = 1;
          // A_x_(3*f+1, 3*f+1) = 1;

          // lb_x_(3*f) = ee_x_des.at(0)(3*f) - 0.03;
          // lb_x_(3*f+1) = ee_x_des.at(0)(3*f+1) - 0.03;
          // ub_x_(3*f) = ee_x_des.at(0)(3*f) + 0.03;
          // ub_x_(3*f+1) = ee_x_des.at(0)(3*f+1) + 0.03;
        }
      }
      tracking_target_ = ee_x_des.at(0);

    }

    vector<VectorXd> u_target;
    for (int i = 0; i < N_; i++) {
      if (ic3_options_.track_ic3_inputs) {
        int idx = std::min(i + best_idx, ic3_options_.N-1);
        u_target.push_back(input_data.col(idx)); 

      // TODO: why does this else exist, isn't it just redundant?
      } else {
        VectorXd gravity(VectorXd::Zero(n_u_));
        // HARDCODED
        if (example_idx_ == 0) {
          gravity[2] = 8.33;
        } else if (example_idx_ == 1 || example_idx_ == 2) {
          gravity[2] = 0.196;
          gravity[5] = 0.196;
          gravity[8] = 0.196;
        }
        u_target.push_back(gravity);
      }
    }

    c3_->UpdateInputTarget(u_target);
  }


  if (ic3_options_.add_position_constraints) {  
    c3_->AddLinearConstraint(A_x_, lb_x_, ub_x_, c3::ConstraintVariable::STATE);
  }
  
  if (ic3_options_.add_input_constraints) {
    c3_->AddLinearConstraint(A_u_, lb_u_, ub_u_, c3::ConstraintVariable::INPUT);  
  }
  c3_->UpdateLCS(lcs);

  auto c3_start = std::chrono::high_resolution_clock::now();
  c3_->Solve(x_lcs);

  c3_->RemoveConstraints();
  
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  double c3_solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(finish - c3_start).count() /
      1e6;
  
  mutable_solve_time[0] = (1 - solve_time_filter_constant_) * solve_time +
                          solve_time_filter_constant_ * mutable_solve_time[0];
  if (controller_options_.publish_frequency > 0) {
    solve_time = 1.0 / controller_options_.publish_frequency;
    mutable_solve_time[0] = solve_time;
  }

  auto z_sol = c3_->GetFullSolution();

  // Print costs
  if (n_u_ == 9) {
    for (int i = 0; i < z_sol.size(); i++) {
      double finger_pos_cost = z_sol[i].segment(0, 9).transpose() * Q_[i].block(0, 0, 9, 9) * z_sol[i].segment(0, 9);
      double cube_rot_cost = z_sol[i].segment(9, 4).transpose() * Q_[i].block(9, 9, 4, 4) * z_sol[i].segment(9, 4);
      double cube_pos_cost = z_sol[i].segment(13, 3).transpose() * Q_[i].block(13, 13, 3, 3) * z_sol[i].segment(13, 3);
      std::cout << "c3 cost " << i << " finger cost " << finger_pos_cost << ", cube rot " 
            << cube_rot_cost << ", cube pos " << cube_pos_cost << std::endl;
    }
    if (value_function_idx >= H.size() || value_function_idx >= g.size()) {
      std::cout << "BAD VALUE FUNCTION IDX HELP " << std::endl;
    }
    VectorXd x_final = c3_->GetFinalStateSolution();
    double final_finger_pos_cost = x_final.segment(0, 9).transpose() * H[value_function_idx].block(0, 0, 9, 9) * x_final.segment(0, 9); 
    double final_cube_rot_cost = x_final.segment(9, 4).transpose() * H[value_function_idx].block(9, 9, 4, 4) * x_final.segment(9, 4);
    double final_cube_pos_cost = x_final.segment(13, 3).transpose() * H[value_function_idx].block(13, 13, 3, 3) * x_final.segment(13, 3);

    std::cout << "c3 cost final "<< " finger cost " << final_finger_pos_cost << ", cube rot " 
            << final_cube_rot_cost << ", cube pos " << final_cube_pos_cost << std::endl;

    final_finger_pos_cost += g[value_function_idx].segment(0, 9).dot(x_final.segment(0, 9));
    final_cube_rot_cost += g[value_function_idx].segment(9, 4).dot(x_final.segment(9, 4));
    final_cube_pos_cost += g[value_function_idx].segment(13, 3).dot(x_final.segment(13, 3));

    std::cout << "c3 cost final with affine "<< " finger cost " << final_finger_pos_cost << ", cube rot " 
            << final_cube_rot_cost << ", cube pos " << final_cube_pos_cost << std::endl;
  }



  if (mutable_solve_time[0] < (N_ - 1) * dt_) {
    int index = mutable_solve_time[0] / dt_;
    double weight = (mutable_solve_time[0] - index * dt_) / dt_;
    mutable_x_pred = (1 - weight) * z_sol[index].segment(0, n_x_) +
                     weight * z_sol[index + 1].segment(0, n_x_);
  } else {
    mutable_x_pred = z_sol[N_ - 1].segment(0, n_x_);
  }

  std::cout << "c3 solve time: " << c3_solve_time << std::endl;

  if (ic3_timestep >= 0 && ic3_timestep <= ic3_options_.N) {
    //std::cout << "ic3_timestep: " << ic3_timestep << ": ";
    //std::cout << c3_->GetInputSolution()[0].transpose() << std::endl;    
  }
  return drake::systems::EventStatus::Succeeded();
}

void iC3TrackingController::UpdateQuaternionCosts(
    const Eigen::VectorXd& x_curr, const Eigen::VectorXd& x_des) const {
    
  // Early return if no quaternions or cost parameters not set
  if (quaternion_indices_.size() == 0 ||
      !controller_options_.quaternion_weight.has_value() ||
      !controller_options_.quaternion_regularizer_fraction.has_value()) {
    return;
  }

  for (int index : quaternion_indices_) {
    Eigen::VectorXd quat_curr_i = x_curr.segment(index, 4).normalized();
    Eigen::VectorXd quat_des_i = x_des.segment(index, 4).normalized();

    Eigen::MatrixXd quat_hessian_i =
        c3::systems::common::hessian_of_squared_quaternion_angle_difference(
                  quat_curr_i, quat_des_i);

    // Regularize hessian so Q is always PSD
    double min_eigenval = quat_hessian_i.eigenvalues().real().minCoeff();
    Eigen::MatrixXd quat_regularizer_1 =
        std::max(0.0, -min_eigenval) * Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd quat_regularizer_2 = quat_des_i * quat_des_i.transpose();

    // Additional regularization term to help with numerical issues
    Eigen::MatrixXd quat_regularizer_3 = 1e-8 * Eigen::MatrixXd::Identity(4, 4);

    double quaternion_weight = controller_options_.quaternion_weight.value();
    double quaternion_regularizer_fraction =
        controller_options_.quaternion_regularizer_fraction.value();

    // Replace quaternion blocks in Q
    double discount_factor = 1;
    for (int i = 0; i < N_ + 1; i++) {
      Q_[i].block(index, index, 4, 4) =
          discount_factor * dt_scaling_ * c3_options_.w_Q * quaternion_weight *
          (quat_hessian_i + quat_regularizer_1 +
           quaternion_regularizer_fraction * quat_regularizer_2 +
           quat_regularizer_3);
      discount_factor *= controller_options_.c3_options.gamma;
    }
  }
}

int iC3TrackingController::GetNearestXForValueFunction(
    VectorXd x_curr, MatrixXd x_hat_slice) const {
  DRAKE_DEMAND(x_hat_slice.cols() > 0);

  int best_idx = 0;
  double best_cost = std::numeric_limits<double>::infinity();

  // HARDCODED INDICES
  int ee_idx;
  int ee_size;
  int object_idx;
  int object_size = 7;

  if (example_idx_ == 0) {
    ee_idx = 0;
    ee_size = 5;
    object_idx = 5;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    ee_idx = 0;
    ee_size = 9;
    object_idx = 9;
  }

  for (int i = 0; i < x_hat_slice.cols(); i++) {
    VectorXd x_diff = x_curr - x_hat_slice.col(i);
    double ee_cost = x_diff.segment(ee_idx, ee_size).transpose() * 
                        ic3_options_.value_function_ee_cost.asDiagonal() * x_diff.segment(ee_idx, ee_size);
                        
    double obj_pos_cost = x_diff.segment(object_idx + 4, 3).transpose() * 
                        ic3_options_.value_function_object_position_cost.asDiagonal() * x_diff.segment(object_idx + 4, 3);
                        
    double velo_cost = x_diff.segment(n_q_, n_v_).transpose() * 
                        ic3_options_.value_function_velocity_cost.asDiagonal() * x_diff.segment(n_q_, n_v_);

    Eigen::Quaterniond q_x_hat(x_hat_slice.col(i)(object_idx), x_hat_slice.col(i)(object_idx+1),
                              x_hat_slice.col(i)(object_idx+2),x_hat_slice.col(i)(object_idx+3));
    Eigen::Quaterniond q_curr(x_curr(object_idx), x_curr(object_idx+1), x_curr(object_idx+2), x_curr(object_idx+3));                     

    double obj_rot_cost = ic3_options_.value_function_object_orientation_cost * q_x_hat.angularDistance(q_curr);


    if (ee_cost + obj_pos_cost + velo_cost + obj_rot_cost < best_cost) {
      best_idx = i;
      best_cost = ee_cost + obj_pos_cost + velo_cost + obj_rot_cost;
    }
  }
  return best_idx;

}

void iC3TrackingController::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  double solve_time = context.get_discrete_state(filtered_solve_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = solve_time + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();

  } 
}


void iC3TrackingController::OutputTrackingTarget(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* tracking_target) const {
	tracking_target->get_mutable_value() = tracking_target_;
}


}  // namespace systems
}  // namespace dairlib
