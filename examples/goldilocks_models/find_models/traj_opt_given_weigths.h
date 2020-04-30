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
    const MatrixXd& B_tau,
    const VectorXd & theta_s, const VectorXd & theta_sDDot,
    double stride_length, double ground_incline, double turning_rate,
    double duration, int n_node, int max_iter,
    double major_optimality_tol, double major_feasibility_tol,
    const std::string& directory, string init_file,
    string  prefix,
    const vector<std::shared_ptr<VectorXd>>& w_sol_vec,
    const vector<std::shared_ptr<MatrixXd>>& A_vec,
    const vector<std::shared_ptr<MatrixXd>>& H_vec,
    const vector<std::shared_ptr<VectorXd>>& y_vec,
    const vector<std::shared_ptr<VectorXd>>& lb_vec,
    const vector<std::shared_ptr<VectorXd>>& ub_vec,
    const vector<std::shared_ptr<VectorXd>>& b_vec,
    const vector<std::shared_ptr<VectorXd>>& c_vec,
    const vector<std::shared_ptr<MatrixXd>>& B_vec,
    const vector<std::shared_ptr<int>>& is_success_vec,
    const vector<std::shared_ptr<int>>& thread_finished_vec,
    double Q_double, double R_double, double all_cost_scale,
    double eps_reg,
    bool is_get_nominal,
    bool is_zero_touchdown_impact,
    bool extend_model,
    bool is_add_tau_in_cost,
    int sample_idx, int n_rerun, double cost_threshold_for_update, int N_rerun,
    int rom_option, int robot_option);

void addRegularization(bool is_get_nominal, double eps_reg,
                       GoldilocksModelTrajOpt& gm_traj_opt);
void setInitialGuessFromFile(const string& directory,
                             const string& init_file,
                             GoldilocksModelTrajOpt& gm_traj_opt);
void augmentConstraintToFixThetaScaling(MatrixXd & B, MatrixXd & A,
                                        VectorXd & y, VectorXd & lb, VectorXd & ub,
                                        int n_s, int n_feature_s,
                                        const VectorXd & theta_s, int sample_idx);

// swing foot pos at mid stance constraint (x and y)
class SwingFootXYPosAtMidStanceConstraint
    : public DirconAbstractConstraint<double> {
 public:
  SwingFootXYPosAtMidStanceConstraint(const MultibodyPlant<double>* plant,
                          const string& body_name,
                          const Vector3d& point_wrt_body)
      : DirconAbstractConstraint<double>(
      2, 3 * plant->num_positions(), VectorXd::Zero(2),
      VectorXd::Zero(2),
      "swing_foot_xy_pos_at_mid_stance_constraint"),
        plant_(plant),
        body_(plant->GetBodyByName(body_name)),
        point_wrt_body_(point_wrt_body) {}
  ~SwingFootXYPosAtMidStanceConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q_0 = x.head(plant_->num_positions());
    VectorXd q_mid = x.segment(plant_->num_positions(), plant_->num_positions());
    VectorXd q_f = x.tail(plant_->num_positions());

    std::unique_ptr<drake::systems::Context<double>> context =
        plant_->CreateDefaultContext();

    VectorX<double> pt_0(3);
    VectorX<double> pt_mid(3);
    VectorX<double> pt_f(3);
    plant_->SetPositions(context.get(), q_0);
    this->plant_->CalcPointsPositions(*context, body_.body_frame(),
                                      point_wrt_body_, plant_->world_frame(),
                                      &pt_0);
    plant_->SetPositions(context.get(), q_mid);
    this->plant_->CalcPointsPositions(*context, body_.body_frame(),
                                      point_wrt_body_, plant_->world_frame(),
                                      &pt_mid);
    plant_->SetPositions(context.get(), q_f);
    this->plant_->CalcPointsPositions(*context, body_.body_frame(),
                                      point_wrt_body_, plant_->world_frame(),
                                      &pt_f);

    *y = (pt_0.head(2) + pt_f.head(2)) - 2 * pt_mid.head(2);
  };

 private:
  const MultibodyPlant<double>* plant_;
  const drake::multibody::Body<double>& body_;
  const Vector3d point_wrt_body_;
};



class ComHeightVelConstraint : public DirconAbstractConstraint<double> {
 public:
  ComHeightVelConstraint(const MultibodyPlant<double>* plant) :
    DirconAbstractConstraint<double>(
      1, 2 * (plant->num_positions() + plant->num_velocities()),
      VectorXd::Zero(1), VectorXd::Zero(1),
      "com_height_vel_constraint"),
    plant_(plant),
    n_q_(plant->num_positions()),
    n_v_(plant->num_velocities()) {

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
    VectorXd v1 = x.segment(n_q_, n_v_);
    VectorXd q2 = x.segment(n_q_ + n_v_, n_q_);
    VectorXd v2 = x.segment(2 * n_q_ + n_v_, n_v_);

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
    plant_->SetPositions(context.get(), q2);
    plant_->SetVelocities(context.get(), v2);
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

  std::vector<BodyIndex> body_indexes_;
  double composite_mass_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
