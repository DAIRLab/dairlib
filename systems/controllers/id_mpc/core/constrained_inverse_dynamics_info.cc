#include <iostream>
#include "constrained_inverse_dynamics_info.h"
#include "common/find_resource.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib::systems::controllers::id_mpc {

using multibody::DistanceEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::PinocchioPlant;

using drake::MatrixX;
using drake::VectorX;
using drake::systems::Context;

using std::make_unique;

ConstrainedDynamicsInfo::ConstrainedDynamicsInfo(std::string urdf):
  urdf_ (urdf) {
  plant_ = make_unique<PinocchioPlant<double>>(0.0, urdf);
  drake::multibody::Parser parser(plant_.get());
  parser.AddModels(FindResourceOrThrow(urdf));
}

void ConstrainedDynamicsInfo::Finalize() {
  plant_->Finalize();
  plant_ad_ = std::make_unique<PinocchioPlant<AutoDiffXd>>(*plant_, urdf_);

  plant_->FinalizePlant();
  plant_ad_->FinalizePlant();

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
  DRAKE_DEMAND(plant_->is_finalized());
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
  DRAKE_DEMAND(plant_->is_finalized());
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
void ConstrainedDynamicsInfo::DoEvaluateKinematics(
    const PinocchioPlant<T> &plant, const Context<T> &context,
    const KinematicEvaluatorSet<T>* holonomic_constraints,
    const ContactConstraintMap<T>& contact_constraint_evaluators,
    const std::vector<std::string> &active_contacts,
    KinematicsResults<T> &eval) const {

  eval.c = VectorX<T>::Zero(nh_ + nc_active_);
  eval.cdot = VectorX<T>::Zero(nh_ + nc_active_);

  MatrixX<T> Jh = MatrixX<T>::Zero(nh_, nv_);

  if (holonomic_constraints != nullptr) {
    eval.c.head(nh_) = holonomic_constraints->EvalFull(context);
    Jh = holonomic_constraints->EvalFullJacobian(context);
  }

  MatrixX<T> Jc_active = MatrixX<T>::Zero(nc_active_, nv_);
  MatrixX<T> Jc = MatrixX<T>::Zero(nc_, nv_);

  for (const auto &c : active_contacts) {
    DRAKE_ASSERT(contact_constraint_evaluators.contains(c));
    const auto &evaluator = contact_constraint_evaluators.at(c);
    Jc.block(lambda_c_start_idxs_.at(c), 0, 3, nv_) =
        evaluator->EvalFullJacobian(context);
    int start = Jc_active_start_idxs_.at(c);

    eval.c.segment(
        nh_ + start, evaluator->num_active()) = evaluator->EvalActive(context);
    for (int i = 0; i < evaluator->num_active(); ++i) {
      Jc_active.row(start + i) =
          Jc.row(lambda_c_start_idxs_.at(c) + evaluator->active_inds().at(i));
    }
  }

  eval.J.topRows(nh_) = Jh;
  eval.J.bottomRows(nc_) = Jc;
  eval.cdot.topRows(nh_) = Jh * plant.GetVelocities(context);
  eval.cdot.bottomRows(nc_active_) = Jc_active * plant.GetVelocities(context);
  plant.MapVelocityToQDot(context, plant.GetVelocities(context), &eval.qdot);
}

template <typename T>
VectorX<T> ConstrainedDynamicsInfo::DoEvaluateInverseDynamics(
    const multibody::PinocchioPlant<T> &plant,
    const Context<T> &context,
    const KinematicsResults<T> &kinematics,
    const VectorX<T> &vdot, const VectorX<T> &lambda) const {

  drake::multibody::MultibodyForces tau_app(plant);
  tau_app.mutable_generalized_forces() = kinematics.J.transpose() * lambda;
  return plant.CalcInverseDynamicsWithGravity(context, vdot, tau_app);

}

template <typename T>
ConstrainedDynamicsInfo::KinematicsResults<T>
ConstrainedDynamicsInfo::MakeEmptyKinematicsResults() const {
  ConstrainedDynamicsInfo::KinematicsResults<T> eval;
  eval.qdot = VectorX<T>::Zero(nq_);
  eval.J = MatrixX<T>::Zero(nh_ + nc_, nv_);
  eval.c = VectorX<T>::Zero(nh_ + nc_active_);
  eval.cdot = VectorX<T>::Zero(nh_ + nc_active_);
  return eval;
}

template<>
VectorX<double> ConstrainedDynamicsInfo::EvaluateInverseDynamics(
    const Context<double>& context,
    const ConstrainedDynamicsInfo::KinematicsResults<double>& kinematics,
    const VectorX<double>& vdot, const VectorX<double>& lambda) const {
  return DoEvaluateInverseDynamics(*plant_, context, kinematics, vdot, lambda);
}

template<>
VectorX<AutoDiffXd> ConstrainedDynamicsInfo::EvaluateInverseDynamics(
    const Context<AutoDiffXd>& context,
    const ConstrainedDynamicsInfo::KinematicsResults<AutoDiffXd>& kinematics,
    const VectorX<AutoDiffXd>& vdot, const VectorX<AutoDiffXd>& lambda) const {
  return DoEvaluateInverseDynamics(
      *plant_ad_, context, kinematics, vdot, lambda);
}

template<>
ConstrainedDynamicsInfo::KinematicsResults<AutoDiffXd>
ConstrainedDynamicsInfo::EvaluateKinematics(
    const Context<AutoDiffXd> &context,
    const std::vector<std::string> &active_contacts) const {
  ConstrainedDynamicsInfo::KinematicsResults<AutoDiffXd> eval =
      MakeEmptyKinematicsResults<AutoDiffXd>();
  DoEvaluateKinematics(
      *plant_ad_, context, holonomic_constraints_ad_.get(),
      contact_constraint_evaluators_ad_, active_contacts, eval);
  return eval;
}

template<>
ConstrainedDynamicsInfo::KinematicsResults<double>
ConstrainedDynamicsInfo::EvaluateKinematics(
    const Context<double> &context,
    const std::vector<std::string> &active_contacts) const {
  ConstrainedDynamicsInfo::KinematicsResults<double> eval =
      MakeEmptyKinematicsResults<double>();
  DoEvaluateKinematics(*plant_, context, holonomic_constraints_.get(),
             contact_constraint_evaluators_, active_contacts, eval);
  return eval;
}

template ConstrainedDynamicsInfo::KinematicsResults<double>
ConstrainedDynamicsInfo::MakeEmptyKinematicsResults() const;

template ConstrainedDynamicsInfo::KinematicsResults<AutoDiffXd>
ConstrainedDynamicsInfo::MakeEmptyKinematicsResults() const;

}