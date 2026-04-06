#include "examples/cube_flip/toy_system/lqr_input.h"
#include <iostream>

using dairlib::lcmt_timestamped_saved_traj;
using dairlib::lcmt_lqr_output;
using drake::systems::BasicVector;

namespace dairlib {

LQRInput::LQRInput(const MultibodyPlant<double>& plant, int N, double dt, int iter_to_use, double time_to_wait) :
  plant_(plant),
  N_(N),
  dt_(dt),
  iter_to_use_(iter_to_use),
  time_to_wait_(time_to_wait) {

  input_port_x_curr_ =
    this->DeclareVectorInputPort("x_curr",
                      plant_.num_positions() + plant_.num_velocities())
        .get_index();

  input_port_ic3_x_ =
    this->DeclareAbstractInputPort("ic3_x_traj",
                      drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  input_port_u_ =
    this->DeclareAbstractInputPort("ic3_u_traj",
                      drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  input_port_lqr_ =
    this->DeclareAbstractInputPort("lqr_input",
                        drake::Value<lcmt_lqr_output>())
          .get_index();

  output_port_index_ =
      this->DeclareVectorOutputPort("u", 5, &LQRInput::ComputeLQRInput)
          .get_index();

}


void LQRInput::ComputeLQRInput(const drake::systems::Context<double>& context,
              drake::systems::BasicVector<double>* output) const {

  double t = context.get_time();
  
  const auto* x_input = this->EvalAbstractInput(context, input_port_ic3_x_);
  const auto* u_input = this->EvalAbstractInput(context, input_port_u_);
  const auto* lqr_input = this->EvalAbstractInput(context, input_port_lqr_);

  const BasicVector<double>* x_curr =
        (BasicVector<double>*)this->EvalVectorInput(context, input_port_x_curr_);
  const auto& lcm_all_x_trajectories = x_input->get_value<lcmt_timestamped_saved_traj>();
	const auto& lcm_all_u_trajectories = u_input->get_value<lcmt_timestamped_saved_traj>();
  const auto& lqr_all_inputs = lqr_input->get_value<lcmt_lqr_output>();


  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);

	const std::string final_trajectory_name = "iteration_" + std::to_string(iter_to_use_);
  Eigen::MatrixXd x_data = x_trajectory.GetTrajectory(final_trajectory_name).datapoints;
  Eigen::MatrixXd u_data = u_trajectory.GetTrajectory(final_trajectory_name).datapoints;

  const std::vector<std::vector<std::vector<double>>>& source_K = lqr_all_inputs.K[iter_to_use_];
  const std::vector<std::vector<double>>& source_k_ff = lqr_all_inputs.k_ff[iter_to_use_];

  vector<Eigen::MatrixXd> K;
  vector<Eigen::VectorXd> k_ff;

  for (int i = 0; i < source_K.size(); i++) {
    // std::cout << "i: " << i << std::endl;
    // std::cout << source_K.size() << std::endl;
    // std::cout << source_K[i].size() << std::endl;
    // std::cout << source_K[i][0].size() << std::endl;
    Eigen::MatrixXd mat(MatrixXd::Zero(source_K[i].size(), source_K[i][0].size()));
    for (int j = 0; j < source_K[i].size(); j++) {
        // std::cout << "j: " << j << std::endl;

        mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_K[i][j].data(), source_K[i][j].size());
    }
    K.push_back(std::move(mat));
    k_ff.push_back(Eigen::VectorXd::Map(source_k_ff[i].data(), source_k_ff[i].size()));
  }
  
  std::unique_ptr<drake::systems::Context<double>> plant_context = plant_.CreateDefaultContext();		
	auto& context_ref = *plant_context;  
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_ref);

  Eigen::VectorXd u_gravity = Eigen::VectorXd::Zero(plant_.num_actuators());

  if (plant_.num_actuators() == 5) {
    u_gravity[2] = -(tau_g[2] + tau_g[10]); // Hard-coded cube + plate
    u_gravity[4] = -(tau_g[4] + 0.13 * tau_g[10]); // Hard-coded cube + plate
  } else if (plant_.num_actuators() == 9) {
    u_gravity[2] = -tau_g[2]; 
    u_gravity[5] = -tau_g[5]; 
    u_gravity[8] = -tau_g[8]; 
  }

  if (t > dt_ * N_ + time_to_wait_ || t < time_to_wait_) { 
    // Just compensate gravity if time is past horizon
    
    if (t - (int)t == 0) {
      std::cout << "time: " << t << std::endl;
    }
    output->SetFromVector(u_gravity);

  } else {

    int k = (int)((t - time_to_wait_) / dt_);

    double alpha = 0;

    VectorXd delta_x = x_curr->get_value() - x_data.col(k);
    VectorXd u = u_data.col(k) + K[k] * delta_x + alpha * k_ff[k];

    std::cout << (t / dt_) << ", " << std::round(t / dt_) << std::endl;
    if ((t / dt_) - std::round(t / dt_) < 1e-3) {
      //std::cout << "delta x: " << delta_x.transpose() << std::endl;
      std::cout << "lqr " << k << ": " << u.transpose() << std::endl;
    }    
    output->SetFromVector(u);

  }
}


} // namespace dairlib