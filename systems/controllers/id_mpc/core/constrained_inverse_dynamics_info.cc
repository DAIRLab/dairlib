#include "constrained_inverse_dynamics_info.h"
#include "common/find_resource.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib::systems::controllers::id_mpc {

using multibody::DistanceEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;

using Eigen::MatrixX;
using drake::VectorX;
using drake::systems::Context;

using std::make_unique;

ConstrainedDynamicsInfo::ConstrainedDynamicsInfo(std::string urdf) {
  plant_ = make_unique<MultibodyPlant<double>>(0.0);
  drake::multibody::Parser parser(plant_.get());
  parser.AddModels(FindResourceOrThrow(urdf));
  plant_->Finalize();
  plant_ad_ = drake::systems::System<double>::ToAutoDiffXd(*plant_);

  holonomic_constraints_ = make_unique<KinematicEvaluatorSet<double>>(*plant_);
  holonomic_constraints_ad_ = make_unique<KinematicEvaluatorSet<AutoDiffXd>>
      (*plant_ad_);

  nq_ = plant_->num_positions();
  nv_ = plant_->num_velocities();
  nu_ = plant_->num_actuators();
}

// TODO (@Brian-Acosta) Put this in the base yaw view frame?
void ConstrainedDynamicsInfo::AddContactPoint(
    std::string name, std::string body,
    const Eigen::Vector3d &point_in_body_frame,
    std::vector<int> active_constraint_directions,
    double friction_coefficient) {
  DRAKE_DEMAND(not contact_constraint_evaluators_.contains(name));
  DRAKE_DEMAND(plant_->HasBodyNamed(body));
  DRAKE_DEMAND(not lambda_c_start_idxs_.contains(name));
  DRAKE_DEMAND(not Jc_active_start_idxs_.contains(name));
  DRAKE_DEMAND(not mu_map_.contains(name));

  contact_constraint_evaluators_.insert({
    name, make_unique<WorldPointEvaluator<double>>(
           *plant_,
           point_in_body_frame,
           plant_->GetBodyByName(body).body_frame(),
           Eigen::Matrix3d::Identity(),
           Eigen::Vector3d::Zero(),
           active_constraint_directions)});

  contact_constraint_evaluators_ad_.insert({
    name, make_unique<WorldPointEvaluator<AutoDiffXd>>(
          *plant_ad_,
          point_in_body_frame,
          plant_ad_->GetBodyByName(body).body_frame(),
          Eigen::Matrix3d::Identity(),
          Eigen::Vector3d::Zero(),
          active_constraint_directions)});

  lambda_c_start_idxs_.insert({name, nc_});
  Jc_active_start_idxs_.insert({name, nc_active_});
  mu_map_.insert({name, friction_coefficient});
  nc_ += contact_constraint_evaluators_.at(name)->num_full();
  nc_active_ += contact_constraint_evaluators_.at(name)->num_active();
}

void ConstrainedDynamicsInfo::AddDistanceConstraint(
    std::string body_A, const Eigen::Vector3d &pt_A,
    std::string body_B, const Eigen::Vector3d &pt_B, double distance) {

  DRAKE_DEMAND(plant_->HasBodyNamed(body_A));
  DRAKE_DEMAND(plant_->HasBodyNamed(body_B));
  DRAKE_DEMAND(distance > 0); // Jacobian is not well-defined for distance 0

  holonomic_constraint_storage_.push_back(
      make_unique<DistanceEvaluator<double>>(
          *plant_,
          pt_A, plant_->GetBodyByName(body_A).body_frame(),
          pt_B, plant_->GetBodyByName(body_B).body_frame(), distance));

  holonomic_constraint_ad_storage_.push_back(
      make_unique<DistanceEvaluator<AutoDiffXd>>(
          *plant_ad_,
          pt_A, plant_ad_->GetBodyByName(body_A).body_frame(),
          pt_B, plant_ad_->GetBodyByName(body_B).body_frame(), distance));

  holonomic_constraints_->add_evaluator(
      holonomic_constraint_storage_.back().get());
  holonomic_constraints_ad_->add_evaluator(
      holonomic_constraint_ad_storage_.back().get());

  nh_ = holonomic_constraints_->count_full();
}

template<>
void ConstrainedDynamicsInfo::SetPlantStateIfNew(
    const VectorX<AutoDiffXd> &x, Context<AutoDiffXd> *context) const {
  multibody::SetPositionsAndVelocitiesIfNew<AutoDiffXd>(*plant_ad_, x, context);
}

template<>
void ConstrainedDynamicsInfo::SetPlantStateIfNew(
    const VectorX<double> &x, Context<double> *context) const {
  multibody::SetPositionsAndVelocitiesIfNew<double>(*plant_, x, context);
}

template <>
std::unique_ptr<Context<double>> ConstrainedDynamicsInfo::MakeContext() const {
  return plant_->CreateDefaultContext();
}

template <>
std::unique_ptr<Context<AutoDiffXd>> ConstrainedDynamicsInfo::MakeContext() const {
  return plant_ad_->CreateDefaultContext();
}

template <typename T>
void ConstrainedDynamicsInfo::DoEvaluate(
    const MultibodyPlant<T> &plant, const Context<T> &context,
    const KinematicEvaluatorSet<T>* holonomic_constraints,
    const ContactConstraintMap<T>& contact_constraint_evaluators,
    const VectorX<T> &vdot, const VectorX<T> &lh, const VectorX<T> &lc,
    const std::vector<std::string> &active_contacts,
    InverseDynamicsEvaluation<T> &eval) const {


  eval.tau_ = VectorX<T>::Zero(nv_);
  eval.c_ = VectorX<T>::Zero(nh_ + nc_active_);
  eval.cdot_ = VectorX<T>::Zero(nh_ + nc_active_);

  MatrixX<T> Jh = MatrixX<T>::Zero(nh_, nv_);
  VectorX<T> JhdotV = VectorX<T>::Zero(nh_);

  if (holonomic_constraints != nullptr) {
    eval.c_.head(nh_) = holonomic_constraints->EvalFull(context);
    Jh = holonomic_constraints->EvalFullJacobian(context);
    JhdotV = holonomic_constraints->EvalFullJacobianDotTimesV(context);
  }

  MatrixX<T> Jc_active = MatrixX<T>::Zero(nc_active_, nv_);
  VectorX<T> Jc_active_dot_v = VectorX<T>::Zero(nc_active_);
  MatrixX<T> Jc = MatrixX<T>::Zero(nc_, nv_);

  for (const auto &c : active_contacts) {
    DRAKE_ASSERT(contact_constraint_evaluators.contains(c));
    const auto &evaluator = contact_constraint_evaluators.at(c);
    Jc.block(lambda_c_start_idxs_.at(c), 0, 3, nv_) =
        evaluator->EvalFullJacobian(context);
    int start = Jc_active_start_idxs_.at(c);

    eval.c_.segment(
        nh_ + start, evaluator->num_active()) = evaluator->EvalActive(context);
    for (int i = 0; i < evaluator->num_active(); ++i) {
      Jc_active.row(start + i) =
          Jc.row(lambda_c_start_idxs_.at(c) + evaluator->active_inds().at(i));
      Jc_active_dot_v.segment(start, evaluator->num_active()) =
          evaluator->EvalActiveJacobianDotTimesV(context);
    }
  }

  VectorX<T> grav = plant.CalcGravityGeneralizedForces(context);

  drake::multibody::MultibodyForces tau_app(plant);
  tau_app.mutable_generalized_forces() = grav + Jc.transpose() * lc + Jh
      .transpose() * lh;

  eval.tau_ = plant.CalcInverseDynamics(context, vdot, tau_app);

  eval.cdot_.head(nh_) = Jh * plant.GetVelocities(context);
  eval.cdot_.tail(nc_active_) = Jc_active * plant.GetVelocities(context);

  eval.qdot_ = VectorX<T>::Zero(nq_);
  plant.MapVelocityToQDot(
      context, plant.GetVelocities(context), &eval.qdot_);
}

template <typename T>
ConstrainedDynamicsInfo::InverseDynamicsEvaluation<T>
ConstrainedDynamicsInfo::MakeEmptyDynamicsEvaluation() const {
  ConstrainedDynamicsInfo::InverseDynamicsEvaluation<T> eval;
  eval.qdot_ = VectorX<T>::Zero(nq_);
  eval.tau_ = VectorX<T>::Zero(nv_);
  eval.c_ = VectorX<T>::Zero(nh_ + nc_active_);
  eval.cdot_ = VectorX<T>::Zero(nh_ + nc_active_);
  return eval;
}

template<>
ConstrainedDynamicsInfo::InverseDynamicsEvaluation<AutoDiffXd>
ConstrainedDynamicsInfo::EvaluateDynamics(
    const Context<AutoDiffXd> &context, const VectorX<AutoDiffXd> &vdot,
    const VectorX<AutoDiffXd> &lh, const VectorX<AutoDiffXd> &lc,
    const std::vector<std::string> &active_contacts) const {

  ConstrainedDynamicsInfo::InverseDynamicsEvaluation<AutoDiffXd> eval;
  DoEvaluate(*plant_ad_, context, holonomic_constraints_ad_.get(),
             contact_constraint_evaluators_ad_, vdot, lh, lc, active_contacts,
             eval);
  return eval;
}

template<>
ConstrainedDynamicsInfo::InverseDynamicsEvaluation<double>
ConstrainedDynamicsInfo::EvaluateDynamics(
    const Context<double> &context, const VectorX<double> &vdot,
    const VectorX<double> &lh, const VectorX<double> &lc,
    const std::vector<std::string> &active_contacts) const {
  ConstrainedDynamicsInfo::InverseDynamicsEvaluation<double> eval;
  DoEvaluate(*plant_, context, holonomic_constraints_.get(),
             contact_constraint_evaluators_, vdot, lh, lc, active_contacts,
             eval);
  return eval;
}

template ConstrainedDynamicsInfo::InverseDynamicsEvaluation<double>
ConstrainedDynamicsInfo::MakeEmptyDynamicsEvaluation() const;

template ConstrainedDynamicsInfo::InverseDynamicsEvaluation<AutoDiffXd>
ConstrainedDynamicsInfo::MakeEmptyDynamicsEvaluation() const;
}