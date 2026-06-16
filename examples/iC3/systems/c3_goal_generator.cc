#include "c3_goal_generator.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using c3::LCS;
using c3::C3Options;
using c3::systems::C3ControllerOptions;
using c3::ContactPairConfig;
using c3::multibody::LCSFactory;

C3GoalGenerator::C3GoalGenerator(
		MultibodyPlant<double>& plant, 
		LCSFactory lcs_factory,
    iC3Options ic3_options, 
		C3ControllerOptions c3_controller_options, 
    VectorXd x_des, int example_idx)
    : plant_(plant), 
			lcs_factory_(lcs_factory),
      ic3_options_(ic3_options),
      c3_controller_options_(c3_controller_options),
			c3_options_(c3_controller_options.c3_options),
			x_des_(x_des),
      N_(c3_controller_options.lcs_factory_options.N),
      example_idx_(example_idx)
  {
  DRAKE_DEMAND(c3_controller_options_.lcs_factory_options.contact_pair_configs.has_value());
		
  this->set_name("c3_goal_generator");
	n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();

	std::cout << "n_x_" << n_x_ << std::endl;

	int n_lambda_with_tangential = 0;
  int num_contacts = c3_controller_options_.lcs_factory_options.contact_pair_configs.value().size();
  for (ContactPairConfig pair : c3_controller_options_.lcs_factory_options.contact_pair_configs.value()) {
    n_lambda_with_tangential += 2 * pair.num_friction_directions;
  }

  if (c3_controller_options_.lcs_factory_options.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * num_contacts + n_lambda_with_tangential;
  } else if (c3_controller_options_.lcs_factory_options.contact_model == "anitescu") {
    n_lambda_ = n_lambda_with_tangential;
  } else {
    std::cerr << ("Unknown or unsupported contact model: " +
      c3_controller_options_.lcs_factory_options.contact_model) << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  for (const auto& body_idx : plant_.GetFloatingBaseBodies()) {
    const auto& body = plant_.get_body(body_idx);
    int start = body.floating_positions_start();
    quaternion_indices_.push_back(start);
  }

  state_port_ =
      this->DeclareVectorInputPort("x_input", TimestampedVector<double>(n_x_))
          .get_index();

  ic3_x_port_ =
      this->DeclareAbstractInputPort("ic3_x_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  timestep_port_ =
      this->DeclareVectorInputPort(
              "timestep_port", BasicVector<double>(1))
          .get_index();

  // HARDCODED
  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    nominal_position_size = 9;
  }
  
  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(nominal_position_size))
          .get_index();

  target_port_ =
      this->DeclareVectorOutputPort(
              "target_port",
							BasicVector<double>(n_x_),
              &C3GoalGenerator::OutputTarget)
          .get_index();

	// Make dummy LCS for port declaration
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_); 
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  LCS dummy_lcs =  LCS(A, B, D, d, E, F, H, c, 
      c3_controller_options_.lcs_factory_options.N, c3_controller_options_.lcs_factory_options.dt);
	
  lcs_port_ =
      this->DeclareAbstractOutputPort(
              "lcs_port",
              dummy_lcs,
              &C3GoalGenerator::OutputLCS)
          .get_index();

  x_lcs_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs_port",
							TimestampedVector<double>(n_x_),
              &C3GoalGenerator::OutputState)
          .get_index();

    

}


void C3GoalGenerator::OutputTarget(
  const Context<double>& context, BasicVector<double>* target) const {
//  std::cout << "C3GoalGenerator::OutputTarget" << std::endl;

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        state_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

	VectorXd xd = x_des_;

	target->get_mutable_value() = xd;
}

void C3GoalGenerator::OutputState(
  const Context<double>& context, TimestampedVector<double>* state_output) const {

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        state_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

	const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

	drake::VectorX<double> x_out = lcs_x->CopyVectorNoTimestamp();
  if (example_idx_ == 0) {
    DRAKE_DEMAND(nominal_position->get_value().size() == 3);
    x_out.segment(0, 3) = x_out.segment(0, 3) - nominal_position->get_value();
    x_out.segment(9, 3) = x_out.segment(9, 3) - nominal_position->get_value();
  }


  state_output->SetDataVector(x_out);
  state_output->set_timestamp(context.get_time());  // Set timestamp.
}

void C3GoalGenerator::OutputLCS(
  const Context<double>& context, LCS* lcs_out) const {

  const BasicVector<double>* nominal_position =
    (BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        state_port_);

  const BasicVector<double>* timestep_vector =
    (BasicVector<double>*)this->EvalVectorInput(context, timestep_port_);                                                       
  int timestep = static_cast<int>(timestep_vector->get_value()(0));

  if (timestep < 0) return;

  std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);

  const auto& lcm_all_x_trajectories = 
    this->EvalAbstractInput(context, ic3_x_port_)->get_value<lcmt_timestamped_saved_traj>();

  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
  MatrixXd state_data = state_trajectory.datapoints;

  drake::VectorX<double> x_lcs = lcs_x->get_data();

  // HARDCODED
  VectorXd u_nominal = VectorXd::Zero(n_u_);
  if (example_idx_ == 0) {
    DRAKE_DEMAND(n_u_ == 5);
    u_nominal(2) = 8.33; // Hard coded plate + object weight
  } else if (example_idx_ == 1 || example_idx_ == 2) {
    DRAKE_DEMAND(n_u_ == 9);
    u_nominal(2) = 0.02 * 9.8;
    u_nominal(5) = 0.02 * 9.8;
    u_nominal(8) = 0.02 * 9.8;
  }
  
  LCS lcs;
  if (ic3_options_.use_time_varying_lcs) {

    MatrixXd x_hat(n_x_, N_);
    MatrixXd u_hat(n_u_, N_);
    double dt_scaling = c3_controller_options_.lcs_factory_options.dt / ic3_options_.dt;

    for (int k = 0; k < N_; k++) {
      int idx = std::min(static_cast<int>(timestep + dt_scaling * k), ic3_options_.N);
      std::cout << "idx " << idx << std::endl;
      x_hat.col(k) = state_data.col(idx);
      u_hat.col(k) = u_nominal;

      // Linearize about true end effector position
      if (example_idx_ == 0) {
        x_hat.col(k).segment(0, 5) = x_lcs.segment(0, 5);
      } else if (example_idx_ == 1 || example_idx_ == 2) {
        x_hat.col(k).segment(0, 9) = x_lcs.segment(0, 9);
      }
    }

    if (example_idx_ == 0) {
      // Translate xyz position to match ic3's origin
      for (int k = 0; k < N_; k++) {
        x_hat.col(k).segment(0, 3) = x_hat.col(k).segment(0, 3) - nominal_position->get_value(); 
        x_hat.col(k).segment(9, 3) = x_hat.col(k).segment(9, 3) - nominal_position->get_value();
      }
    }

    lcs = MakeTimeVaryingLCS(x_hat, u_hat);

  } else {
    if (example_idx_ == 0) {
      // Translate xyz position to match ic3's origin
      x_lcs.segment(0, 3) = x_lcs.segment(0, 3) - nominal_position->get_value(); 
      x_lcs.segment(9, 3) = x_lcs.segment(9, 3) - nominal_position->get_value();
    }

    // Normalize quaternions
    for (auto idx : quaternion_indices_) {
      x_lcs.segment(idx, 4) = x_lcs.segment(idx, 4).normalized();
    }

    if (!x_lcs.allFinite()) {
      std::cout << "c3 goal gen x_lcs not all finite " << x_lcs.transpose() << std::endl;
    }

    lcs_factory_.SetNewDt(c3_controller_options_.lcs_factory_options.dt);
    lcs_factory_.UpdateStateAndInput(x_lcs, u_nominal);
    lcs = lcs_factory_.GenerateLCS();
  }

	*lcs_out = lcs;
}



LCS C3GoalGenerator::MakeTimeVaryingLCS(MatrixXd x_hat, MatrixXd u_hat) const {

  DRAKE_DEMAND(x_hat.cols() >= N_);
  DRAKE_DEMAND(u_hat.cols() >= N_);

  vector<Eigen::MatrixXd> A;
  vector<Eigen::MatrixXd> B;
  vector<Eigen::MatrixXd> D;
  vector<Eigen::VectorXd> d;
  vector<Eigen::MatrixXd> E;
  vector<Eigen::MatrixXd> F;
  vector<Eigen::MatrixXd> H;
  vector<Eigen::VectorXd> c;

  for (int k = 0; k < N_; k++) {
    for (auto idx : quaternion_indices_) {
      x_hat.col(k).segment(idx, 4) = x_hat.col(k).segment(idx, 4).normalized(); // Normalize quaternions
    }

    // Linearize about kth xhat, uhat
    lcs_factory_.UpdateStateAndInput(x_hat.col(k), u_hat.col(k));
    LCS lcs = lcs_factory_.GenerateLCS();
    A.push_back(lcs.A()[0]);
    B.push_back(lcs.B()[0]);
    D.push_back(lcs.D()[0]);
    d.push_back(lcs.d()[0]);
    E.push_back(lcs.E()[0]);
    F.push_back(lcs.F()[0]);
    H.push_back(lcs.H()[0]);
    c.push_back(lcs.c()[0]);      
  }
  return LCS(A, B, D, d, E, F, H, c, c3_controller_options_.lcs_factory_options.dt);
}

} // namespace dairlib