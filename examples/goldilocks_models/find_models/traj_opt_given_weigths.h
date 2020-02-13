#pragma once

#include <string>
#include <Eigen/Dense>
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

#include "multibody/multibody_utils.h"

#include "examples/goldilocks_models/find_models/goldilocks_model_traj_opt.h"

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using drake::multibody::MultibodyPlant;
using drake::AutoDiffXd;

using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;

using drake::math::RotationMatrix;
using drake::math::RollPitchYaw;

using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;

namespace dairlib {
namespace goldilocks_models  {

void trajOptGivenWeights(
  const MultibodyPlant<double> & plant,
  const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
  int n_s, int n_sDDot, int n_tau, int n_feature_s, int n_feature_sDDot,
  MatrixXd B_tau,
  const VectorXd & theta_s, const VectorXd & theta_sDDot,
  double stride_length, double ground_incline, double duration, int max_iter,
  double major_optimality_tol, double major_feasibility_tol,
  vector<double> var_scale,
  std::string directory, std::string init_file, std::string prefix,
  /*vector<VectorXd> * w_sol_vec,
  vector<MatrixXd> * A_vec, vector<MatrixXd> * H_vec,
  vector<VectorXd> * y_vec,
  vector<VectorXd> * lb_vec, vector<VectorXd> * ub_vec,
  vector<VectorXd> * b_vec,
  vector<VectorXd> * c_vec,
  vector<MatrixXd> * B_vec,*/
  double Q_double, double R_double,
  double eps_reg,
  bool is_get_nominal,
  bool is_zero_touchdown_impact,
  bool extend_model,
  bool is_add_tau_in_cost,
  int batch,
  int robot_option);

void addRegularization(bool is_get_nominal, double eps_reg,
                       GoldilocksModelTrajOpt& gm_traj_opt);
void setInitialGuessFromFile(const VectorXd& w_sol,
                             GoldilocksModelTrajOpt& gm_traj_opt);
void augmentConstraintToFixThetaScaling(MatrixXd & B, MatrixXd & A,
                                        VectorXd & y, VectorXd & lb, VectorXd & ub,
                                        int n_s, int n_feature_s,
                                        const VectorXd & theta_s, int batch);

class QuaternionNormConstraint : public DirconAbstractConstraint<double> {
 public:
  QuaternionNormConstraint(vector<double> var_scale) :
    DirconAbstractConstraint<double>(1, 4,
                                     VectorXd::Zero(1), VectorXd::Zero(1),
                                     "quaternion_norm_constraint"),
    quaternion_scale_(var_scale[4]) {
  }
  ~QuaternionNormConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorX<double> output(1);
    output << quaternion_scale_ * x.norm() - 1;
    *y = output;
  };
 private:
  double quaternion_scale_;
};

class LeftFootYConstraint : public DirconAbstractConstraint<double> {
 public:
  LeftFootYConstraint(const MultibodyPlant<double>* plant,
                      vector<double> var_scale) :
    DirconAbstractConstraint<double>(
      1, plant->num_positions(),
      VectorXd::Ones(1) * 0.03,
      VectorXd::Ones(1) * std::numeric_limits<double>::infinity(),
      "left_foot_constraint"),
    plant_(plant),
    body_(plant->GetBodyByName("toe_left")),
    quaternion_scale_(var_scale[4]) {
  }
  ~LeftFootYConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    q.head(4) *= quaternion_scale_;

    std::unique_ptr<drake::systems::Context<double>> context =
          plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q);

    VectorX<double> pt(3);
    this->plant_->CalcPointsPositions(*context,
                                      body_.body_frame(), Vector3d::Zero(),
                                      plant_->world_frame(), &pt);
    *y = pt.segment(1, 1);
  };
 private:
  const MultibodyPlant<double>* plant_;
  const drake::multibody::Body<double>& body_;
  double quaternion_scale_;
};
class RightFootYConstraint : public DirconAbstractConstraint<double> {
 public:
  RightFootYConstraint(const MultibodyPlant<double>* plant,
                       vector<double> var_scale) :
    DirconAbstractConstraint<double>(
      1, plant->num_positions(),
      VectorXd::Ones(1) * (-std::numeric_limits<double>::infinity()),
      VectorXd::Ones(1) * (-0.03),
      "right_foot_constraint"),
    plant_(plant),
    body_(plant->GetBodyByName("toe_right")),
    quaternion_scale_(var_scale[4]) {
  }
  ~RightFootYConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    q.head(4) *= quaternion_scale_;

    std::unique_ptr<drake::systems::Context<double>> context =
          plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q);

    VectorX<double> pt(3);
    this->plant_->CalcPointsPositions(*context,
                                      body_.body_frame(), Vector3d::Zero(),
                                      plant_->world_frame(), &pt);
    *y = pt.segment(1, 1);
  };
 private:
  const MultibodyPlant<double>* plant_;
  const drake::multibody::Body<double>& body_;
  double quaternion_scale_;
};
class RightFootZConstraint : public DirconAbstractConstraint<double> {
 public:
  RightFootZConstraint(const MultibodyPlant<double>* plant,
                       double ground_incline,
                       vector<double> var_scale) :
    DirconAbstractConstraint<double>(
      1, plant->num_positions(),
      VectorXd::Ones(1) * 0.05,
      VectorXd::Ones(1) * std::numeric_limits<double>::infinity(),
      "right_foot_height_constraint"),
    plant_(plant),
    body_(plant->GetBodyByName("toe_right")),
    quaternion_scale_(var_scale[4]) {

    Eigen::AngleAxisd rollAngle(0, Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ground_incline, Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0, Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d inv_rot_mat_ground = q.matrix().transpose();

    T_ground_incline_ = inv_rot_mat_ground;
  }
  ~RightFootZConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    q.head(4) *= quaternion_scale_;

    std::unique_ptr<drake::systems::Context<double>> context =
          plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q);

    VectorX<double> pt(3);
    this->plant_->CalcPointsPositions(*context,
                                      body_.body_frame(), Vector3d::Zero(),
                                      plant_->world_frame(), &pt);
    *y = (T_ground_incline_ * pt).tail(1);
  };
 private:
  const MultibodyPlant<double>* plant_;
  const drake::multibody::Body<double>& body_;
  double quaternion_scale_;

  Eigen::Matrix3d T_ground_incline_;
};

class ComHeightVelConstraint : public DirconAbstractConstraint<double> {
 public:
  ComHeightVelConstraint(const MultibodyPlant<double>* plant,
                         vector<double> var_scale) :
    DirconAbstractConstraint<double>(
      1, 2 * (plant->num_positions() + plant->num_velocities()),
      VectorXd::Zero(1), VectorXd::Zero(1),
      "com_height_vel_constraint"),
    plant_(plant),
    n_q_(plant->num_positions()),
    n_v_(plant->num_velocities()),
    omega_scale_(var_scale[0]),
    quaternion_scale_(var_scale[4]) {

    DRAKE_DEMAND(plant->num_bodies() > 1);
    DRAKE_DEMAND(plant->num_model_instances() > 1);

    // Get all body indices
    std::vector<ModelInstanceIndex> model_instances;
    for (ModelInstanceIndex model_instance_index(1);
         model_instance_index < plant->num_model_instances();
         ++model_instance_index)
      model_instances.push_back(model_instance_index);
    for (auto model_instance : model_instances) {
      const std::vector<BodyIndex> body_index_in_instance =
        plant->GetBodyIndices(model_instance);
      for (BodyIndex body_index : body_index_in_instance)
        body_indexes_.push_back(body_index);
    }
    // Get total mass
    std::unique_ptr<drake::systems::Context<double>> context =
          plant->CreateDefaultContext();
    for (BodyIndex body_index : body_indexes_) {
      if (body_index == 0) continue;
      const Body<double>& body = plant_->get_body(body_index);

      // Calculate composite_mass_.
      const double& body_mass = body.get_mass(*context);
      // composite_mass_ = ∑ mᵢ
      composite_mass_ += body_mass;
    }
    if (!(composite_mass_ > 0)) {
      throw std::runtime_error(
        "The total mass must larger than zero.");
    }
  }
  ~ComHeightVelConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q1 = x.head(n_q_);
    q1.head(4) *= quaternion_scale_;
    VectorXd v1 = x.segment(n_q_, n_v_) * omega_scale_;
    VectorXd q2 = x.segment(n_q_ + n_v_, n_q_);
    q2.head(4) *= quaternion_scale_;
    VectorXd v2 = x.segment(2 * n_q_ + n_v_, n_v_) * omega_scale_;

    std::unique_ptr<drake::systems::Context<double>> context =
          plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q1);
    plant_->SetVelocities(context.get(), v1);

    const drake::multibody::Frame<double>& world = plant_->world_frame();

    // Get com jacobian for x1
    MatrixXd Jcom1 = MatrixXd::Zero(3, n_v_);
    for (BodyIndex body_index : body_indexes_) {
      if (body_index == 0) continue;

      const Body<double>& body = plant_->get_body(body_index);
      const Vector3d pi_BoBcm = body.CalcCenterOfMassInBodyFrame(*context);

      // Calculate M * J in world frame.
      const double& body_mass = body.get_mass(*context);
      // Jcom = ∑ mᵢ * Ji
      MatrixXd Jcom_i(3, n_v_);
      plant_->CalcJacobianTranslationalVelocity(
        *context, drake::multibody::JacobianWrtVariable::kV,
        body.body_frame(), pi_BoBcm, world, world, &Jcom_i);
      Jcom1 += body_mass * Jcom_i;
      // cout << "body_mass = " << body_mass << endl;
      // cout << "Jcom_i = " << Jcom_i << endl;
    }
    Jcom1 /= composite_mass_;

    // Get com jacobian for x2
    plant_->SetPositions(context.get(), q1);
    plant_->SetVelocities(context.get(), v1);
    MatrixXd Jcom2 = MatrixXd::Zero(3, n_v_);
    for (BodyIndex body_index : body_indexes_) {
      if (body_index == 0) continue;

      const Body<double>& body = plant_->get_body(body_index);
      const Vector3d pi_BoBcm = body.CalcCenterOfMassInBodyFrame(*context);

      // Calculate M * J in world frame.
      const double& body_mass = body.get_mass(*context);
      // Jcom = ∑ mᵢ * Ji
      MatrixXd Jcom_i(3, n_v_);
      plant_->CalcJacobianTranslationalVelocity(
        *context, drake::multibody::JacobianWrtVariable::kV,
        body.body_frame(), pi_BoBcm, world, world, &Jcom_i);
      Jcom2 += body_mass * Jcom_i;
      // cout << "body_mass = " << body_mass << endl;
      // cout << "Jcom_i = " << Jcom_i << endl;
    }
    Jcom2 /= composite_mass_;


    *y = Jcom1.row(2) * v1 - Jcom2.row(2) * v2;
  };
 private:
  const MultibodyPlant<double>* plant_;
  int n_q_;
  int n_v_;
  double omega_scale_;
  double quaternion_scale_;

  std::vector<BodyIndex> body_indexes_;
  double composite_mass_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
