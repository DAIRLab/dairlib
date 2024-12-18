#include "constrained_dynamics_info.h"
#include "common/find_resource.h"
#include "multibody/kinematic/distance_evaluator.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib::systems::controllers::id_mpc {

using multibody::DistanceEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;

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
    std::vector<int> active_constraint_diretions) {
  DRAKE_DEMAND(not contact_constraint_evaluators_.contains(name));
  DRAKE_DEMAND(plant_->HasBodyNamed(body));

  contact_constraint_evaluators_.insert({
    name, make_unique<WorldPointEvaluator<double>>(
           *plant_,
           point_in_body_frame,
           plant_->GetBodyByName(body).body_frame())});
  contact_constraint_evaluators_ad_.insert({
    name, make_unique<WorldPointEvaluator<AutoDiffXd>>(
          *plant_ad_,
          point_in_body_frame,
          plant_ad_->GetBodyByName(body).body_frame())});

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

}