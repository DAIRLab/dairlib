#include "c3_tracking_system.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using solvers::LCS;
using solvers::LCSFactory;

C3TrackingSystem::C3TrackingSystem(
		MultibodyPlant<double>& plant, 
		Context<double>* context,
		MultibodyPlant<AutoDiffXd>& plant_ad,
		Context<AutoDiffXd>* context_ad,
		vector<SortedPair<GeometryId>>& contact_geoms,
		C3Options c3_options)
    : plant_(plant), 
			context_(context),
			plant_ad_(plant_ad),
			context_ad_(context_ad),
			contact_geoms_(contact_geoms),
			c3_options_(std::move(c3_options)) 
  {
		
  this->set_name("c3_tracking_system");

	n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();
	n_lambda_with_tangential_ = 2 * c3_options_.num_friction_directions * c3_options_.num_contacts;

  curr_x_port_ =
    this->DeclareAbstractInputPort("curr_x_port",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  curr_u_port_ =
      this->DeclareAbstractInputPort("curr_u_port",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  target_port_ =
      this->DeclareVectorOutputPort(
              "target_port",
							BasicVector<double>(n_x_),
              &C3TrackingSystem::OutputTarget)
          .get_index();

	// Make dummy LCS for port declaration
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_with_tangential_);
  MatrixXd E = MatrixXd::Zero(n_lambda_with_tangential_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_with_tangential_, n_lambda_with_tangential_);
  MatrixXd H = MatrixXd::Zero(n_lambda_with_tangential_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_with_tangential_);
  LCS dummy_lcs =  LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.dt);
	
  lcs_port_ =
      this->DeclareAbstractOutputPort(
              "lcs_port",
              dummy_lcs,
              &C3TrackingSystem::OutputLCS)
          .get_index();

}


void C3TrackingSystem::OutputTarget(
  const Context<double>& context, BasicVector<double>* target) const {

  const auto* x_input = this->EvalAbstractInput(context, curr_x_port_);
	const auto& lcm_x_trajectory = x_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory x_trajectory = LcmTrajectory(lcm_x_trajectory.saved_traj);

	MatrixXd x_hat = x_trajectory.GetTrajectory("current_x_trajectory").datapoints;
	target->get_mutable_value() = x_hat.col(x_hat.cols()-1);
}

void C3TrackingSystem::OutputLCS(
  const Context<double>& context, LCS* lcs) const {
  
  const auto* x_input = this->EvalAbstractInput(context, curr_x_port_);
	const auto& lcm_x_trajectory = x_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory x_trajectory = LcmTrajectory(lcm_x_trajectory.saved_traj);

  const auto* u_input = this->EvalAbstractInput(context, curr_u_port_);
	const auto& lcm_u_trajectory = u_input->get_value<lcmt_timestamped_saved_traj>();
	LcmTrajectory u_trajectory = LcmTrajectory(lcm_u_trajectory.saved_traj);

	MatrixXd x_hat = x_trajectory.GetTrajectory("current_x_trajectory").datapoints;
	MatrixXd u_hat = u_trajectory.GetTrajectory("current_u_trajectory").datapoints;

	*lcs = MakeTimeVaryingLCS(x_hat, u_hat);

}

LCS C3TrackingSystem::MakeTimeVaryingLCS(
	MatrixXd x_hat, MatrixXd u_hat) const {

	vector<Eigen::MatrixXd> A;
	vector<Eigen::MatrixXd> B;
	vector<Eigen::MatrixXd> D;
	vector<Eigen::VectorXd> d;
	vector<Eigen::MatrixXd> E;
	vector<Eigen::MatrixXd> F;
	vector<Eigen::MatrixXd> H;
	vector<Eigen::VectorXd> c;

	for (int k = 0; k < c3_options_.N; k++) {
		
		// Set plant to kth xhat, uhat
		multibody::SetContext<double>(plant_, x_hat.col(k), u_hat.col(k),  context_);
		drake::VectorX<double> q_v_u(n_x_ + n_u_);
		q_v_u << x_hat.col(k), u_hat.col(k);
		drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);
		multibody::SetPositionsAndVelocitiesIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.head(n_x_),
																							context_ad_);
		multibody::SetInputsIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u_), context_ad_);

		// Linearize
		vector<int> starting_index_per_contact_in_lambda_t_vector;
		vector<int> num_friction_directions_vector;
		for (int i = 0; i < c3_options_.num_contacts; i++) {
			starting_index_per_contact_in_lambda_t_vector.push_back(i);
			num_friction_directions_vector.push_back(c3_options_.num_friction_directions);
		}

		LCS lcs = LCSFactory::LinearizePlantToLCS(plant_, *context_, plant_ad_, 
			*context_ad_, contact_geoms_, c3_options_.mu, c3_options_.dt, 1, n_lambda_with_tangential_,
			num_friction_directions_vector, starting_index_per_contact_in_lambda_t_vector);


		A.push_back(lcs.A_[0]);
		B.push_back(lcs.B_[0]);
		D.push_back(lcs.D_[0]);
		d.push_back(lcs.d_[0]);
		E.push_back(lcs.E_[0]);
		F.push_back(lcs.F_[0]);
		H.push_back(lcs.H_[0]);
		c.push_back(lcs.c_[0]);      
	}

	return LCS(A, B, D, d, E, F, H, c, c3_options_.dt);
}

} // namespace dairlib