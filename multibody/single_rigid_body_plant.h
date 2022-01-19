#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

enum BipedStance {
  kLeft=0,
  kRight=1
};

namespace multibody {

  Eigen::Matrix3d HatOperator3x3(const Eigen::Vector3d& v);

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
    void SetMass(double mass) {mass_ = mass;}
    Eigen::VectorXd CalcSRBStateFromPlantState(const Eigen::VectorXd& x) const;
    std::vector<Eigen::Vector3d> CalcFootPositions(const Eigen::VectorXd& x) const;
    Eigen::Vector3d CalcFootPosition (const Eigen::VectorXd& x, const BipedStance& stance) const;

    void CopyContinuousLinearized3dSrbDynamicsForMPC(
        double m, double yaw, BipedStance stance,
        const Eigen::MatrixXd &b_I,
        const Eigen::Vector3d &eq_com_pos,
        const Eigen::Vector3d &eq_foot_pos,
        const Eigen::Vector3d &eq_lambda,
        const Eigen::Vector2d &eq_tq_yaw_pitch,
        const drake::EigenPtr<Eigen::MatrixXd> &A,
        const drake::EigenPtr<Eigen::MatrixXd> &B,
        const drake::EigenPtr<Eigen::VectorXd> &b);

    void CopyDiscreteLinearizedSrbDynamicsForMPC(
        double dt, double m, double yaw, BipedStance stance,
        const Eigen::MatrixXd &b_I,
        const Eigen::Vector3d &eq_com_pos,
        const Eigen::Vector3d &eq_foot_pos,
        const Eigen::Vector3d &eq_lambda,
        const Eigen::Vector2d &eq_tq_yaw_pitch,
        const drake::EigenPtr<Eigen::MatrixXd> &Ad,
        const drake::EigenPtr<Eigen::MatrixXd> &Bd,
        const drake::EigenPtr<Eigen::VectorXd> &bd);

    void AddBaseFrame(
        const std::string &body_name,
        const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

    void AddContactPoint(
        const std::pair<
        Eigen::Vector3d,
        const drake::multibody::BodyFrame<double>&>& pt,
        BipedStance stance);

    int nu() const { return nu_; }
    int nq() const { return nq_; }
    int nv() const { return nv_; }
    double mass() const { return mass_; }


   private:

    const bool use_com_;
    int nq_;
    int nv_;
    int nu_;

    const int nx_ = 12;

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

}}