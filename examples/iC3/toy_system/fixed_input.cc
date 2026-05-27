#include "examples/iC3/toy_system/fixed_input.h"
#include <iostream>

using dairlib::lcmt_timestamped_saved_traj;

namespace dairlib {

FixedInput::FixedInput(const MultibodyPlant<double>& plant, int N, double dt, int iter_to_use, double time_to_wait) :
  plant_(plant),
  N_(N),
  dt_(dt),
  iter_to_use_(iter_to_use),
  time_to_wait_(time_to_wait) {

  input_port_index_ =
      this->DeclareAbstractInputPort("ic3_u_traj",
                        drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  output_port_index_ =
      this->DeclareVectorOutputPort("u", plant_.num_actuators(), &FixedInput::ComputeFixedInput)
          .get_index();

}


void FixedInput::ComputeFixedInput(const drake::systems::Context<double>& context,
              drake::systems::BasicVector<double>* output) const {

  double t = context.get_time();
  

  const auto* u_input = this->EvalAbstractInput(context, input_port_index_);
	const auto& lcm_all_u_trajectories = u_input->get_value<lcmt_timestamped_saved_traj>();

  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);
  
  std::vector<std::string> traj_names = u_trajectory.GetTrajectoryNames();
  // for (int i = 0; i < traj_names.size(); i++) {
  //   std::cout << traj_names.at(i) << std::endl;
  // }

	const std::string final_trajectory_name = "iteration_" + std::to_string(iter_to_use_);
  Eigen::MatrixXd u_data = u_trajectory.GetTrajectory(final_trajectory_name).datapoints;

  std::unique_ptr<drake::systems::Context<double>> plant_context = plant_.CreateDefaultContext();		
	auto& context_ref = *plant_context;  
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_ref);

  Eigen::VectorXd u_gravity = Eigen::VectorXd::Zero(plant_.num_actuators());
  // HARDCODED FOR EXAMPLES
  if (plant_.num_actuators() == 5) { 
    u_gravity[2] = -(tau_g[2] + tau_g[10]); // Hard-coded cube + plate
    if (t < time_to_wait_) {
      u_gravity[4] = -(0.13 * tau_g[10]); // Hard-coded cube + plate
    }
    std::cout << u_gravity.transpose() << std::endl;

  } else if (plant_.num_actuators() == 9) {
    u_gravity[2] = -(tau_g[2]); // Hard-coded fingertip
    u_gravity[5] = -(tau_g[5]); // Hard-coded fingertip
    u_gravity[8] = -(tau_g[8]); // Hard-coded fingertip
    // u_gravity[11] = -(tau_g[11]); // Hard-coded fingertip

  }
  

  

  if (t > dt_ * N_ + time_to_wait_ || t < time_to_wait_) { 
    // Just compensate gravity if time is past horizon
    
    if (t - (int)t == 0) {
      std::cout << "time: " << t << std::endl;
    }
    output->SetFromVector(u_gravity);

  } else {

    int k = (int)((t - time_to_wait_) / dt_);
    // std::cout << "u: " << u_data.col(k).transpose() << std::endl;
    if (std::abs(((t - time_to_wait_) / dt_) - std::floor((t - time_to_wait_) / dt_)) < 1e-4) {
      std::cout << "open " << k << ": " << u_data.col(k).transpose() << std::endl;
    }
    
    output->SetFromVector(u_data.col(k));

  }
}


} // namespace dairlib