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
		Context<double>* context,
		MultibodyPlant<AutoDiffXd>& plant_ad,
		Context<AutoDiffXd>* context_ad,
		C3ControllerOptions c3_controller_options, 
    VectorXd x_des, int example_idx)
    : plant_(plant), 
			context_(context),
			plant_ad_(plant_ad),
			context_ad_(context_ad),
      c3_controller_options_(c3_controller_options),
			c3_options_(c3_controller_options.c3_options),
			x_des_(x_des),
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

  lcs_factory_ = std::make_unique<LCSFactory>(plant_, *context_, plant_ad_, *context_ad_, c3_controller_options_.lcs_factory_options);

  state_port_ =
      this->DeclareVectorInputPort("x_input", TimestampedVector<double>(n_x_))
          .get_index();

  // HARDCODED
  int nominal_position_size;
  if (example_idx_ == 0) {
    nominal_position_size = 3;
  } else if (example_idx_ == 2) {
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

  x_lcs_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs_port",
							TimestampedVector<double>(n_x_),
              &C3GoalGenerator::OutputState)
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

	VectorXd x_out = lcs_x->get_value();
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
  drake::VectorX<double> x_lcs = lcs_x->get_data();


  if (example_idx_ == 0) {
    // Translate xyz position to match ic3's origin
    x_lcs.segment(0, 3) = x_lcs.segment(0, 3) - nominal_position->get_value(); 
    x_lcs.segment(9, 3) = x_lcs.segment(9, 3) - nominal_position->get_value();
  }

   // HARDCODED
	VectorXd u_nominal = VectorXd::Zero(n_u_);
  if (example_idx_ == 0) {
    DRAKE_DEMAND(n_u_ == 5);
    u_nominal(2) = 5; // Hard coded plate + object weight
  } else if (example_idx_ == 2) {
    DRAKE_DEMAND(n_u_ == 9);
    u_nominal(2) = 0.2 * 9.8;
    u_nominal(5) = 0.2 * 9.8;
    u_nominal(8) = 0.2 * 9.8;
  }

	c3::multibody::SetContext<double>(plant_, x_lcs, u_nominal,  context_);
	drake::VectorX<double> q_v_u(n_x_ + n_u_);
	q_v_u << x_lcs, u_nominal;
	drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);
	c3::multibody::SetPositionsAndVelocitiesIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.head(n_x_),
																						context_ad_);
	c3::multibody::SetInputsIfNew<AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u_), context_ad_);

	// Linearize
	LCS lcs = lcs_factory_->GenerateLCS();

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