#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {

enum BipedStance {
  kLeft=0,
  kRight=1
};

/// SingleRigidBodyPlant represents a biped robot as a single rigid body
/// floating base with massless legs and point feet.

class SingleRigidBodyPlant {

 public:
  SingleRigidBodyPlant(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      bool use_com);
  double CalcMassFromListOfBodies(const std::vector<std::string>& bodies);
  double SetMassFromListOfBodies(const std::vector<std::string>& bodies);
  Eigen::VectorXd CalcSRBStateFromPlantState(const Eigen::VectorXd& x) const;
  std::vector<Eigen::Vector3d> CalcFootPositions(const Eigen::VectorXd& x);


  void AddBaseFrame(
      const std::string &body_name,
      const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

  void AddContactPoint(
      std::pair<Eigen::Vector3d, const drake::multibody::BodyFrame<double>&> pt,
      BipedStance stance);




 private:

  const bool use_com_;
  int nq_;
  int nv_;

  const int nx_ = 7 + 6;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_frame_;
  std::vector<std::pair<Eigen::Vector3d,
      const drake::multibody::BodyFrame<double>&>> contact_points_;
  double mass_;

  mutable drake::math::RollPitchYaw<double> rpy_;
  mutable drake::systems::Context<double>* plant_context_;

  std::string base_;
  Eigen::Vector3d com_offset_;

};
}