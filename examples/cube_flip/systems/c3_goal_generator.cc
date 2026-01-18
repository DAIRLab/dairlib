#include "c3_goal_generator.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using solvers::LCS;
using solvers::LCSFactory;
using systems::TimestampedVector;

C3GoalGenerator::C3GoalGenerator(
		MultibodyPlant<double>& plant, 
		Context<double>* context,
		MultibodyPlant<AutoDiffXd>& plant_ad,
		Context<AutoDiffXd>* context_ad,
		vector<SortedPair<GeometryId>>& contact_geoms,
		C3Options c3_options, VectorXd x_des)
    : plant_(plant), 
			context_(context),
			plant_ad_(plant_ad),
			context_ad_(context_ad),
			contact_geoms_(contact_geoms),
			c3_options_(std::move(c3_options)),
			x_des_(x_des)
  {
		
  this->set_name("c3_goal_generator");

	n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_u_ = plant_.num_actuators();

	std::cout << "n_x_" << n_x_ << std::endl;

	int n_lambda_with_tangential = 2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
    n_lambda_ =
        2 * c3_options_.num_contacts + n_lambda_with_tangential;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model_ = solvers::ContactModel::kAnitescu;
    n_lambda_ = n_lambda_with_tangential;
  } else {
    std::cerr << ("Unknown or unsupported contact model: " +
      c3_options_.contact_model) << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  state_port_ =
      this->DeclareVectorInputPort("x_input", TimestampedVector<double>(n_x_))
          .get_index();

  nominal_position_port_ =
      this->DeclareVectorInputPort(
              "nominal_position", BasicVector<double>(3))
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
  LCS dummy_lcs =  LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.dt);
	
  lcs_port_ =
      this->DeclareAbstractOutputPort(
              "lcs_port",
              dummy_lcs,
              &C3GoalGenerator::OutputLCS)
          .get_index();

}


void C3GoalGenerator::OutputTarget(
  const Context<double>& context, BasicVector<double>* target) const {
//  std::cout << "C3GoalGenerator::OutputTarget" << std::endl;

	const BasicVector<double>* nominal_position =
			(BasicVector<double>*)this->EvalVectorInput(context, nominal_position_port_);

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        state_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

	// hard coded, set z goal to nominal z
	VectorXd xd = x_des_;
	xd.segment(0, 2) = x_lcs.segment(9, 2);
	xd(2) = nominal_position->get_value()(2);

	xd.segment(9, 3) = nominal_position->get_value();

	target->get_mutable_value() = xd;
}

void C3GoalGenerator::OutputLCS(
  const Context<double>& context, LCS* lcs_out) const {
  
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        state_port_);
  drake::VectorX<double> x_lcs = lcs_x->get_data();

	VectorXd u_nominal = VectorXd::Zero(n_u_);
	u_nominal(2) = 53; // Hard coded plate + object weight

	multibody::SetContext<double>(plant_, x_lcs, u_nominal,  context_);
	drake::VectorX<double> q_v_u(n_x_ + n_u_);
	q_v_u << x_lcs, u_nominal;
	drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);
	multibody::SetPositionsAndVelocitiesIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.head(n_x_),
																						context_ad_);
	multibody::SetInputsIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u_), context_ad_);

	// Linearize
	vector<int> starting_index_per_contact_in_lambda_t_vector;
	vector<int> num_friction_directions_vector;
	for (int i = 0; i < c3_options_.num_contacts; i++) {
		starting_index_per_contact_in_lambda_t_vector.push_back(2 * c3_options_.num_friction_directions * i);
		num_friction_directions_vector.push_back(c3_options_.num_friction_directions);
	}

	LCS lcs = LCSFactory::LinearizePlantToLCS(plant_, *context_, plant_ad_, 
		*context_ad_, contact_geoms_, c3_options_.mu, c3_options_.dt, c3_options_.N, n_lambda_,
		num_friction_directions_vector, starting_index_per_contact_in_lambda_t_vector, contact_model_);

	*lcs_out = lcs;
}

// LCS C3GoalGenerator::MakeTimeVaryingLCS(
// 	MatrixXd x_hat, MatrixXd u_hat) const {

// 	vector<Eigen::MatrixXd> A;
// 	vector<Eigen::MatrixXd> B;
// 	vector<Eigen::MatrixXd> D;
// 	vector<Eigen::VectorXd> d;
// 	vector<Eigen::MatrixXd> E;
// 	vector<Eigen::MatrixXd> F;
// 	vector<Eigen::MatrixXd> H;
// 	vector<Eigen::VectorXd> c;

// 	for (int k = 0; k < c3_options_.N; k++) {
		
// 		// Set plant to kth xhat, uhat
// 		multibody::SetContext<double>(plant_, x_hat.col(k), u_hat.col(k),  context_);
// 		drake::VectorX<double> q_v_u(n_x_ + n_u_);
// 		q_v_u << x_hat.col(k), u_hat.col(k);
// 		drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);
// 		multibody::SetPositionsAndVelocitiesIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.head(n_x_),
// 																							context_ad_);
// 		multibody::SetInputsIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u_), context_ad_);

// 		// Linearize
// 		vector<int> starting_index_per_contact_in_lambda_t_vector;
// 		vector<int> num_friction_directions_vector;
// 		for (int i = 0; i < c3_options_.num_contacts; i++) {
// 			starting_index_per_contact_in_lambda_t_vector.push_back(2 * c3_options_.num_friction_directions * i);
// 			num_friction_directions_vector.push_back(c3_options_.num_friction_directions);
// 		}

// 		LCS lcs = LCSFactory::LinearizePlantToLCS(plant_, *context_, plant_ad_, 
// 			*context_ad_, contact_geoms_, c3_options_.mu, c3_options_.dt, 1, n_lambda_,
// 			num_friction_directions_vector, starting_index_per_contact_in_lambda_t_vector, contact_model_);


// 		A.push_back(lcs.A_[0]);
// 		B.push_back(lcs.B_[0]);
// 		D.push_back(lcs.D_[0]);
// 		d.push_back(lcs.d_[0]);
// 		E.push_back(lcs.E_[0]);
// 		F.push_back(lcs.F_[0]);
// 		H.push_back(lcs.H_[0]);
// 		c.push_back(lcs.c_[0]);      
// 	}

// 	return LCS(A, B, D, d, E, F, H, c, c3_options_.dt);
// }

} // namespace dairlib