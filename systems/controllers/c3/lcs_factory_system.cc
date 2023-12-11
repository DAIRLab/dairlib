#include "lcs_factory_system.h"

#include <utility>

#include "multibody/multibody_utils.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using solvers::LCS;
using solvers::LCSFactory;
using systems::TimestampedVector;

namespace systems {

LCSFactorySystem::LCSFactorySystem(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
        contact_geoms,
    C3Options c3_options)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      c3_options_(std::move(c3_options)),
      N_(c3_options_.N) {
  this->set_name("lcs_factory_system");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  dt_ = c3_options_.dt;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
            2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  }
  n_u_ = plant_.num_actuators();

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();

//  lcs_inputs_input_port_ =
//      this->DeclareVectorInputPort("u_lcs", BasicVector<double>(n_u_))
//          .get_index();

  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Zero(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  lcs_port_ = this->DeclareAbstractOutputPort(
                      "lcs", LCS(A, B, D, d, E, F, H, c, N_, dt_),
                      &LCSFactorySystem::OutputLCS)
                  .get_index();

  lcs_contact_jacobian_port_ = this->DeclareAbstractOutputPort(
                      "J_lcs", Eigen::MatrixXd(n_x_, n_lambda_),
                      &LCSFactorySystem::OutputLCSContactJacobian)
                  .get_index();

  lcs_contact_points_port_ = this->DeclareAbstractOutputPort(
                      "p_lcs", std::vector<Eigen::VectorXd>(),
                      &LCSFactorySystem::OutputLCSContactPoints)
                  .get_index();
}

void LCSFactorySystem::OutputLCS(const drake::systems::Context<double>& context,
                                 LCS* output_lcs) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
//  const auto lcs_u =
//      (BasicVector<double>*)this->EvalVectorInput(context,
//                                                  lcs_inputs_input_port_);
  DRAKE_DEMAND(lcs_x->get_data().size() == n_x_);
//  DRAKE_DEMAND(lcs_u->get_value().size() == n_u_);
  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
                     plant_.num_actuators());
//  q_v_u << lcs_x->get_data(), lcs_u->get_value();
  q_v_u << lcs_x->get_data(), VectorXd::Zero(n_u_);
  drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x_));
  multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u_), context_);
  multibody::SetInputsIfNew<drake::AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u_),
                                               context_ad_);
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  double scale;
  std::tie(*output_lcs, scale) = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N, contact_model);
}

void LCSFactorySystem::OutputLCSContactJacobian(const drake::systems::Context<double>& context,
                                                Eigen::MatrixXd* output_jacobian) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
          plant_.num_actuators());
  // u is irrelevant in pure geometric/kinematic calculation
  q_v_u << lcs_x->get_data(), VectorXd::Zero(n_u_);

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x_));
  multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u_), context_);
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  std::vector<Eigen::VectorXd> contact_points;
  std::tie(*output_jacobian, contact_points) = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N, contact_model);
}

void LCSFactorySystem::OutputLCSContactPoints(const drake::systems::Context<double>& context,
                                              std::vector<Eigen::VectorXd>* contact_points) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
          plant_.num_actuators());
  // u is irrelevant in pure geometric/kinematic calculation
  q_v_u << lcs_x->get_data(), VectorXd::Zero(n_u_);

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x_));
  multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u_), context_);
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  MatrixXd contact_jacobian;
  contact_points->clear();
  std::tie(contact_jacobian, *contact_points) = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N, contact_model);

//  for (auto& contact_point : witness_points_){
//    contact_points->push_back(contact_point);
//  }
}



}  // namespace systems
}  // namespace dairlib
