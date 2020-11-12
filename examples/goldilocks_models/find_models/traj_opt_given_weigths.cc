#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"

#include <chrono>
#include <memory>
#include <string>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/attic/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_helper_func.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "solvers/nonlinear_cost.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::solvers::NonlinearConstraint;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using dairlib::systems::trajectory_optimization::PointVelocityConstraint;

using drake::AutoDiffXd;
using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>

namespace dairlib::goldilocks_models {

// swing foot pos at mid stance constraint (x and y)
class SwingFootXYPosAtMidStanceConstraint : public NonlinearConstraint<double> {
 public:
  SwingFootXYPosAtMidStanceConstraint(const MultibodyPlant<double>* plant,
                                      const string& body_name,
                                      const Vector3d& point_wrt_body)
      : NonlinearConstraint<double>(
            2, 3 * plant->num_positions(), VectorXd::Zero(2), VectorXd::Zero(2),
            "swing_foot_xy_pos_at_mid_stance_constraint"),
        plant_(plant),
        body_(plant->GetBodyByName(body_name)),
        point_wrt_body_(point_wrt_body) {}
  ~SwingFootXYPosAtMidStanceConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q_0 = x.head(plant_->num_positions());
    VectorXd q_mid =
        x.segment(plant_->num_positions(), plant_->num_positions());
    VectorXd q_f = x.tail(plant_->num_positions());

    std::unique_ptr<drake::systems::Context<double>> context =
        plant_->CreateDefaultContext();

    drake::VectorX<double> pt_0(3);
    drake::VectorX<double> pt_mid(3);
    drake::VectorX<double> pt_f(3);
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

class ComHeightVelConstraint : public NonlinearConstraint<double> {
 public:
  ComHeightVelConstraint(const MultibodyPlant<double>* plant)
      : NonlinearConstraint<double>(
            1, 2 * (plant->num_positions() + plant->num_velocities()),
            VectorXd::Zero(1), VectorXd::Zero(1), "com_height_vel_constraint"),
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
      const drake::multibody::Body<double>& body = plant_->get_body(body_index);

      // Calculate composite_mass_.
      const double& body_mass = body.get_mass(*context);
      // composite_mass_ = ∑ mᵢ
      composite_mass_ += body_mass;
    }
    if (!(composite_mass_ > 0)) {
      throw std::runtime_error("The total mass must larger than zero.");
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

      const drake::multibody::Body<double>& body = plant_->get_body(body_index);
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

      const drake::multibody::Body<double>& body = plant_->get_body(body_index);
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

class ComZeroHeightVelConstraint : public NonlinearConstraint<double> {
 public:
  ComZeroHeightVelConstraint(const MultibodyPlant<double>& plant)
      : NonlinearConstraint<double>(
            1, plant.num_positions() + plant.num_velocities(),
            VectorXd::Zero(1), VectorXd::Zero(1),
            "com_zero_height_vel_constraint"),
        plant_(plant),
        world_(plant.world_frame()),
        context_(plant.CreateDefaultContext()),
        n_q_(plant.num_positions()),
        n_v_(plant.num_velocities()) {}
  ~ComZeroHeightVelConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x.head(n_q_);
    VectorXd v = x.tail(n_v_);
    plant_.SetPositions(context_.get(), q);
    plant_.SetVelocities(context_.get(), v);

    MatrixX<double> J_com(3, n_v_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, world_, world_, &J_com);

    *y = J_com.row(2) * v;
  };

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  int n_q_;
  int n_v_;
};

class CollocationVelocityCost : public solvers::NonlinearCost<double> {
 public:
  CollocationVelocityCost(const MatrixXd& W_Q,
                          const drake::multibody::MultibodyPlant<double>& plant,
                          DirconKinematicDataSet<double>* constraints,
                          const std::string& description = "")
      : solvers::NonlinearCost<double>(
            1 + 2 * (plant.num_positions() + plant.num_velocities()) +
                (2 * plant.num_actuators()) +
                (2 * constraints->countConstraintsWithoutSkipping()),
            description),
        plant_(plant),
        context_(plant_.CreateDefaultContext()),
        constraints_(constraints),
        n_v_(plant.num_velocities()),
        n_x_(plant.num_positions() + plant.num_velocities()),
        n_u_(plant.num_actuators()),
        n_lambda_(constraints->countConstraintsWithoutSkipping()),
        W_Q_(W_Q){};

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>>& x,
                    drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == 1 + 2 * (n_x_ + n_u_ + n_lambda_));

    // Extract our input variables:
    // h - current time (knot) value
    // x0, x1 state vector at time steps k, k+1
    // u0, u1 input vector at time steps k, k+1
    const auto h = x(0);
    const auto x0 = x.segment(1, n_x_);
    const auto x1 = x.segment(1 + n_x_, n_x_);
    const auto u0 = x.segment(1 + (2 * n_x_), n_u_);
    const auto u1 = x.segment(1 + (2 * n_x_) + n_u_, n_u_);
    const auto l0 = x.segment(1 + 2 * (n_x_ + n_u_), n_lambda_);
    const auto l1 = x.segment(1 + 2 * (n_x_ + n_u_) + n_lambda_, n_lambda_);

    multibody::setContext<double>(plant_, x0, u0, context_.get());
    constraints_->updateData(*context_, l0);
    const VectorX<double> xdot0 = constraints_->getXDot();

    multibody::setContext<double>(plant_, x1, u1, context_.get());
    constraints_->updateData(*context_, l1);
    const VectorX<double> xdot1 = constraints_->getXDot();

    // Cubic interpolation to get xcol and xdotcol.
    const VectorX<double> xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
    //    const VectorX<double> xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 +
    //    xdot1);

    (*y) = xcol.tail(n_v_).transpose() * W_Q_ * xcol.tail(n_v_);
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  DirconKinematicDataSet<double>* constraints_;

  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;

  MatrixXd W_Q_;
};

class JointAccelCost : public solvers::NonlinearCost<double> {
 public:
  JointAccelCost(const MatrixXd& W_Q,
                 const drake::multibody::MultibodyPlant<double>& plant,
                 DirconKinematicDataSet<double>* constraints,
                 const std::string& description = "")
      : solvers::NonlinearCost<double>(
            plant.num_positions() + plant.num_velocities() +
                plant.num_actuators() +
                constraints->countConstraintsWithoutSkipping(),
            description),
        plant_(plant),
        context_(plant_.CreateDefaultContext()),
        constraints_(constraints),
        n_v_(plant.num_velocities()),
        n_x_(plant.num_positions() + plant.num_velocities()),
        n_u_(plant.num_actuators()),
        n_lambda_(constraints->countConstraintsWithoutSkipping()),
        W_Q_(W_Q){};

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>>& x,
                    drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == n_x_ + n_u_ + n_lambda_);

    // Extract our input variables:
    const auto x0 = x.segment(0, n_x_);
    const auto u0 = x.segment(n_x_, n_u_);
    const auto l0 = x.segment(n_x_ + n_u_, n_lambda_);

    multibody::setContext<double>(plant_, x0, u0, context_.get());
    constraints_->updateData(*context_, l0);
    const VectorX<double> xdot0 = constraints_->getXDot();

    (*y) = xdot0.tail(n_v_).transpose() * W_Q_ * xdot0.tail(n_v_);
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  DirconKinematicDataSet<double>* constraints_;

  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;

  MatrixXd W_Q_;
};

class JointAccelConstraint : public solvers::NonlinearConstraint<double> {
 public:
  JointAccelConstraint(const VectorXd& lb, const VectorXd& ub,
                       const drake::multibody::MultibodyPlant<double>& plant,
                       DirconKinematicDataSet<double>* constraints,
                       const std::string& description = "")
      : solvers::NonlinearConstraint<double>(
            plant.num_velocities(),
            plant.num_positions() + plant.num_velocities() +
                plant.num_actuators() +
                constraints->countConstraintsWithoutSkipping(),
            lb, ub, description),
        plant_(plant),
        context_(plant_.CreateDefaultContext()),
        constraints_(constraints),
        n_v_(plant.num_velocities()),
        n_x_(plant.num_positions() + plant.num_velocities()),
        n_u_(plant.num_actuators()),
        n_lambda_(constraints->countConstraintsWithoutSkipping()){};

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == n_x_ + n_u_ + n_lambda_);

    // Extract our input variables:
    const auto x0 = x.segment(0, n_x_);
    const auto u0 = x.segment(n_x_, n_u_);
    const auto l0 = x.segment(n_x_ + n_u_, n_lambda_);

    multibody::setContext<double>(plant_, x0, u0, context_.get());
    constraints_->updateData(*context_, l0);

    (*y) = constraints_->getXDot().tail(n_v_);
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  DirconKinematicDataSet<double>* constraints_;

  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;

  MatrixXd W_Q_;
};

void addRegularization(bool is_get_nominal, double eps_reg,
                       GoldilocksModelTrajOpt& gm_traj_opt) {
  // Add regularization term here so that hessian is pd (for outer loop), so
  // that we can use schur complement method
  // TODO(yminchen): should I add to all decision variable? or just state?
  if (!is_get_nominal) {
    auto w = gm_traj_opt.dircon->decision_variables();
    for (int i = 0; i < w.size(); i++)
      gm_traj_opt.dircon->AddQuadraticCost(eps_reg * w(i) * w(i));
    // cout << "w = " << w << endl;
    // cout << "Check the order of decisiion variable: \n";
    // for (int i = 0; i < w.size(); i++)
    //   cout << gm_traj_opt.dircon->FindDecisionVariableIndex(w(i)) << endl;
  }
}

void setInitialGuessFromFile(const string& directory, const string& init_file,
                             GoldilocksModelTrajOpt& gm_traj_opt) {
  VectorXd w0 = readCSV(directory + init_file).col(0);
  int n_dec = gm_traj_opt.dircon->decision_variables().rows();
  if (n_dec > w0.rows()) {
    cout << "dim(initial guess) < dim(decision var). "
            "Fill the rest with zero's.\n";
    VectorXd old_w0 = w0;
    w0.resize(n_dec);
    w0 = VectorXd::Zero(n_dec);
    w0.head(old_w0.rows()) = old_w0;
  }
  gm_traj_opt.dircon->SetInitialGuessForAllVariables(w0);
}

void augmentConstraintToFixThetaScaling(MatrixXd& B, MatrixXd& A, VectorXd& y,
                                        VectorXd& lb, VectorXd& ub, int n_y,
                                        int n_feature_s,
                                        const VectorXd& theta_y,
                                        int sample_idx) {
  // sum theta over a row = const

  int n_c = B.rows();
  int n_t = B.cols();
  int n_w = A.cols();

  MatrixXd B_old = B;
  B.resize(n_c + n_y, n_t);
  B = MatrixXd::Zero(n_c + n_y, n_t);
  B.block(0, 0, n_c, n_t) = B_old;
  for (int i = 0; i < n_y; i++) {
    B.block(n_c + i, i * n_feature_s, 1, n_feature_s) =
        VectorXd::Ones(n_feature_s).transpose();
  }

  MatrixXd A_old = A;
  A.resize(n_c + n_y, n_w);
  A = MatrixXd::Zero(n_c + n_y, n_w);
  A.block(0, 0, n_c, n_w) = A_old;

  MatrixXd y_old = y;
  y.resize(n_c + n_y);
  VectorXd y_append = VectorXd::Zero(n_y);
  for (int i = 0; i < n_y; i++) {
    for (int j = 0; j < n_feature_s; j++) {
      y_append(i) += theta_y(j + i * n_feature_s);
    }
  }
  y << y_old, y_append;

  MatrixXd lb_old = lb;
  lb.resize(n_c + n_y);
  lb << lb_old, y_append;

  MatrixXd ub_old = ub;
  ub.resize(n_c + n_y);
  ub << ub_old, y_append;

  if (sample_idx == 0)
    cout << "parameters sum per position = " << y_append.transpose() << endl;
}

void extractResult(VectorXd& w_sol, GoldilocksModelTrajOpt& gm_traj_opt,
                   const MathematicalProgramResult& result,
                   std::chrono::duration<double> elapsed,
                   std::vector<int> num_time_samples, int N,
                   const MultibodyPlant<double>& plant,
                   const MultibodyPlant<AutoDiffXd>& plant_autoDiff,
                   const InnerLoopSetting& setting,
                   const ReducedOrderModel& rom, const Task& task,
                   const SubQpData& QPs, int sample_idx, int n_rerun,
                   double cost_threshold_for_update, int N_rerun,
                   vector<DirconKinematicDataSet<double>*> dataset_list,
                   bool is_print_for_debugging) {
  string directory = setting.directory;
  string prefix = setting.prefix;

  // Extract solution
  SolutionResult solution_result = result.get_solution_result();
  double tau_cost = 0;
  if (setting.is_add_tau_in_cost) {
    // Way 1
    for (auto const& binding : gm_traj_opt.tau_cost_bindings) {
      const auto& tau_i = binding.variables();
      VectorXd tau_i_double = result.GetSolution(tau_i);
      VectorXd y_val(1);
      binding.evaluator()->Eval(tau_i_double, &y_val);
      tau_cost += y_val(0);
    }
  }

  /*cout << "sample_idx# = " << sample_idx << endl;
  cout << "    stride_length = " << stride_length << " | "
       << "ground_incline = " << ground_incline << " | "
       << "init_file = " << init_file << endl;
  cout << "    Solve time:" << elapsed.count() << " | ";
  cout << solution_result <<  " | ";
  cout << "Cost:" << result.get_optimal_cost() <<
       " (tau cost = " << tau_cost << ")\n";*/
  /*string string_to_print = "sample #" + to_string(sample_idx) +
                           " (rerun #" + to_string(n_rerun) + ")"
                           "\n    stride_length = " + to_string(stride_length) +
                           " | ground_incline = " + to_string(ground_incline) +
                           " | init_file = " + init_file +
                           "\n    Solve time:" + to_string(elapsed.count()) +
                           " | " + to_string(solution_result) +
                           " | Cost:" + to_string(result.get_optimal_cost()) +
                           " (tau cost = " + to_string(tau_cost) + ")\n";*/
  string string_to_print =
      "  " + to_string(sample_idx) + " (" + to_string(n_rerun) + ")";
  for (auto& task : task.get()) {
    string_to_print += " | " + to_string(task);
  }
  string_to_print +=
      " | " + setting.init_file + " | " + to_string(solution_result) + " | " +
      to_string(elapsed.count()) + " | " +
      to_string(result.get_optimal_cost()) + " (" + to_string(tau_cost) + ")\n";
  cout << string_to_print;

  // Check which solver we are using
  // cout << "Solver: " << result.get_solver_id().name() << endl;

  // Reminder: if you change the algorithm in the next few lines, you also need
  // to change the one in postProcessing
  cout << "(sample_idx, n_rerun, N_rerun, is_success) = (" << sample_idx << ", "
       << n_rerun << ", " << N_rerun << ", " << result.is_success() << ")\n";
  if (n_rerun > N_rerun) {
    if ((cost_threshold_for_update ==
         std::numeric_limits<double>::infinity()) &&
        (to_string(solution_result) == "IterationLimit")) {
      // Do nothing. Continue to store the result
      cout << "#" << sample_idx
           << " hit iteration limit, and hasn't found a solution in this "
              "iteration yet. Will continue solving\n";
    } else if (!result.is_success()) {
      cout << "the rerun of idx #" << sample_idx
           << " was not successful, skip\n";
      return;
    } else if (result.get_optimal_cost() > cost_threshold_for_update) {
      cout << "the cost of idx #" << sample_idx
           << " is higher than before, skip\n";
      return;
    }
  }

  cout << "storing result...\n";

  VectorXd is_success(1);
  if (result.is_success())
    is_success << 1;
  else if (to_string(solution_result) == "IterationLimit")
    is_success << 0.5;
  else
    is_success << 0;
  writeCSV(directory + prefix + string("is_success.csv"), is_success);

  *(QPs.is_success_vec[sample_idx]) = result.is_success() ? 1 : 0;

  // bool constraint_satisfied = solvers::CheckGenericConstraints(*trajopt,
  // result, 1e-5); cout << "constraint_satisfied = " << constraint_satisfied <<
  // endl;

  // Store time, state, and its derivatives for cubic spline reconstruction
  VectorXd t_cubic_spline;
  MatrixXd x_cubic_spline;
  MatrixXd xdot_cubic_spline;
  gm_traj_opt.ConstructStateCubicSplineInfo(
      result, plant, num_time_samples, dataset_list, &t_cubic_spline,
      &x_cubic_spline, &xdot_cubic_spline);
  writeCSV(directory + prefix + string("t_cubic_spline.csv"), t_cubic_spline);
  writeCSV(directory + prefix + string("x_cubic_spline.csv"), x_cubic_spline);
  writeCSV(directory + prefix + string("xdot_cubic_spline.csv"),
           xdot_cubic_spline);

  // Get the solution of all the decision variable
  w_sol = result.GetSolution(gm_traj_opt.dircon->decision_variables());
  writeCSV(directory + prefix + string("w.csv"), w_sol);
  // if (result.is_success())
  //   writeCSV(directory + prefix + string("w (success).csv"), w_sol);
  if (is_print_for_debugging) {
    for (int i = 0; i < w_sol.size(); i++) {
      cout << i << ": " << gm_traj_opt.dircon->decision_variables()[i] << ", "
           << w_sol[i] << endl;
    }
    cout << endl;
  }

  // Store the time, state, and input at knot points
  // TODO: store states from different modes into different files
  VectorXd time_at_knots = gm_traj_opt.dircon->GetSampleTimes(result);
  MatrixXd state_at_knots = gm_traj_opt.dircon->GetStateSamples(result);
  MatrixXd input_at_knots = gm_traj_opt.dircon->GetInputSamples(result);
  auto xf = gm_traj_opt.dircon->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);
  //  state_at_knots.col(N - 1) = result.GetSolution(xf);
  // cout << "you'll need to update state_at_knots if it's multiple modes\n";
  writeCSV(directory + prefix + string("time_at_knots.csv"), time_at_knots);
  writeCSV(directory + prefix + string("state_at_knots.csv"), state_at_knots);
  writeCSV(directory + prefix + string("input_at_knots.csv"), input_at_knots);
  if (is_print_for_debugging) {
    cout << "time_at_knots = \n" << time_at_knots << "\n";
    cout << "state_at_knots = \n" << state_at_knots << "\n";
    //  cout << "state_at_knots.size() = " << state_at_knots.size() << endl;
    cout << "input_at_knots = \n" << input_at_knots << "\n";
  }

  // Also store lambda. We might need to look at it in the future!
  // (save it so we don't need to rerun)
  std::ofstream ofile;
  ofile.open(directory + prefix + string("lambda_at_knots.txt"),
             std::ofstream::out);
  // cout << "lambda_at_knots = \n";
  for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambdai = gm_traj_opt.dircon->force(mode, index);
      ofile << result.GetSolution(lambdai).transpose() << endl;
      if (is_print_for_debugging) {
        cout << result.GetSolution(lambdai).transpose() << endl;
      }
    }
  }
  ofile.close();

  VectorXd c(1);
  c << result.get_optimal_cost();
  VectorXd c_without_tau(1);
  c_without_tau << c(0) - tau_cost;
  writeCSV(directory + prefix + string("c.csv"), c);
  writeCSV(directory + prefix + string("c_without_tau.csv"), c_without_tau);

  QPs.c_vec[sample_idx]->resizeLike(c);
  *(QPs.c_vec[sample_idx]) = c;

  // Testing
  /*bool is_check_tau = true;
  if (is_check_tau) {
    int N_accum = 0;
    for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
      for (int m = 0; m < num_time_samples[l] - 1 ; m++) {
        int i = N_accum + m;
        cout << "i = " << i << endl;
        // Get tau_append
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
        auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
        auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
        auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd tau_i_sol = result.GetSolution(tau_i);
        VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
        VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
        VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

        VectorXd tau_append_head =
          gm_traj_opt.dynamics_constraint_at_head->computeTauToExtendModel(
            x_i_sol, x_iplus1_sol, h_i_sol, theta_y);
        cout << "tau_head = " << tau_append_head.transpose() << endl;
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }
  }*/

  // Testing: print out the timesteps
  // for(int i = 0; i<N-1 ; i++){
  //   auto h_i = gm_traj_opt.dircon->timestep(i);
  //   VectorXd h_i_sol = gm_traj_opt.dircon->GetSolution(h_i);
  //   cout << "h_"<< i <<"_sol = " << h_i_sol << endl;
  // }
  /*auto h_0 = gm_traj_opt.dircon->timestep(0);
  VectorXd h_0_sol = result.GetSolution(h_0);
  cout << "timestep = " << h_0_sol << endl << endl;*/

  // Testing: print out the vertical pos
  /*for(int i = 0; i<N ; i++){
    auto x_i = gm_traj_opt.dircon->state(i);
    VectorXd x_i_sol = result.GetSolution(x_i);
    cout << "x1_"<< i <<"_sol = " << x_i_sol(1) << endl;
  }*/

  // Testing: see what are the decision varaibles
  // cout << "\nAll decision variable:\n"
  //   << gm_traj_opt.dircon->decision_variables() << endl;

  // Testing - see the solution values (to decide how big we should scale them)
  // VectorXd z = result.GetSolution(
  //                    gm_traj_opt.dircon->decision_variables());
  // for(int i = 0; i<z.size(); i++){
  //   cout << gm_traj_opt.dircon->decision_variables()[i] << ", " << z[i] <<
  //   endl;
  // }
  // cout << endl;
}

void postProcessing(const VectorXd& w_sol, GoldilocksModelTrajOpt& gm_traj_opt,
                    const MathematicalProgramResult& result,
                    std::vector<int> num_time_samples, int N,
                    const MultibodyPlant<double>& plant,
                    const MultibodyPlant<AutoDiffXd>& plant_autoDiff,
                    const InnerLoopSetting& setting,
                    const ReducedOrderModel& rom, const SubQpData& QPs,
                    bool is_get_nominal, bool extend_model, int sample_idx,
                    int n_rerun, double cost_threshold_for_update, int N_rerun,
                    int rom_option, int robot_option) {
  int n_q = plant.num_positions();
  int n_y = rom.n_y();
  int n_yddot = rom.n_y();
  int n_tau = rom.n_tau();
  int n_feature_s = rom.n_feature_y();
  const VectorXd& theta_y = rom.theta_y();
  const VectorXd& theta_yddot = rom.theta_yddot();

  string directory = setting.directory;
  string prefix = setting.prefix;

  if (is_get_nominal || !result.is_success()) {
    // Do nothing.
  } else if (extend_model && (n_rerun == N_rerun)) {  // Extending the model
    VectorXd theta_y_append =
        readCSV(directory + string("theta_y_append.csv")).col(0);
    int n_extend = theta_y_append.rows() / n_feature_s;

    // Update trajectory optimization solution
    // Assume that tau is at the end of the decision variable
    VectorXd tau_new = VectorXd::Zero((n_tau + n_extend) * N);
    int N_accum = 0;
    for (unsigned int l = 0; l < num_time_samples.size(); l++) {
      for (int m = 0; m < num_time_samples[l] - 1; m++) {
        int i = N_accum + m;
        // cout << "i = " << i << endl;
        // Get tau_append
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
        auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
        VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

        VectorXd tau_append_head =
            gm_traj_opt.dynamics_constraint_at_head->computeTauToExtendModel(
                x_i_sol, x_iplus1_sol, h_i_sol, theta_y_append);
        // cout << "tau_append_head = " << tau_append_head.transpose() << endl;
        // VectorXd tau_append_tail =
        //   gm_traj_opt.dynamics_constraint_at_tail->computeTauToExtendModel(
        //     x_i_sol, x_iplus1_sol, h_i_sol, theta_y_append);
        // cout << "tau_append_tail = " << tau_append_tail.transpose() << endl;

        // Update tau
        tau_new.segment(i * (n_tau + n_extend) + n_tau, n_extend) +=
            tau_append_head;
        // tau_new.segment((i + 1) * (n_tau + n_extend) + n_tau, n_extend) +=
        //   tau_append_tail;
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }
    // store new w_sol
    VectorXd w_sol_new(w_sol.rows() + n_extend * N);
    w_sol_new << w_sol.head(w_sol.rows() - n_tau * N), tau_new;
    writeCSV(directory + prefix + string("w (no extension).csv"), w_sol);
    writeCSV(directory + prefix + string("w.csv"), w_sol_new);

    // Create a file that shows the new index of theta_yddot
    // Assume that the new features include all the old features (in dynamics)
    VectorXd prime_numbers = createPrimeNumbers(2 * (n_y + n_extend));

    int new_rom_option = 0;
    if (rom_option == 0) {
      new_rom_option = 1;
    } else if (rom_option == 2) {
      new_rom_option = 3;
    } else {
      DRAKE_DEMAND(false);
    }
    DynamicsExpression dyn_expr_old(n_yddot, 0, rom_option, robot_option);
    DynamicsExpression dyn_expr_new(n_yddot + n_extend, 0, new_rom_option,
                                    robot_option);
    VectorXd dummy_s_new = prime_numbers.head(n_y + n_extend);
    VectorXd dummy_yddot_new = prime_numbers.tail(n_y + n_extend);
    VectorXd dummy_s_old = dummy_s_new.head(n_y);
    VectorXd dummy_yddot_old = dummy_yddot_new.head(n_y);
    VectorXd feat_old = dyn_expr_old.getFeature(dummy_s_old, dummy_yddot_old);
    VectorXd feat_new = dyn_expr_new.getFeature(dummy_s_new, dummy_yddot_new);

    VectorXd new_idx(feat_old.rows());
    for (int i = 0; i < feat_old.rows(); i++) {
      int idx = -1;
      for (int j = 0; j < feat_new.rows(); j++) {
        if (feat_old(i) == feat_new(j)) idx = j;
      }
      if (idx == -1) cout << "Failed to create the index list automatically.\n";

      DRAKE_DEMAND(idx > -1);
      new_idx(i) = idx;
    }
    writeCSV(directory + string("theta_yddot_new_index.csv"), new_idx);

  } else {
    if (n_rerun > N_rerun) {
      if (!result.is_success()) {
        return;
      } else if (result.get_optimal_cost() > cost_threshold_for_update) {
        return;
      }
    }

    // Assume theta is fixed. Get the linear approximation of
    //      // the cosntraints and second order approximation of the cost.
    // cout << "\nGetting A, H, y, lb, ub, b.\n";
    MatrixXd A, H;
    VectorXd y, lb, ub, b;
    // cout << "LinearizeConstraints...\n";
    solvers::LinearizeConstraints(*gm_traj_opt.dircon, w_sol, &y, &A, &lb, &ub);
    // cout << "SecondOrderCost...\n";
    solvers::SecondOrderCost(*gm_traj_opt.dircon, w_sol, &H, &b);

    // Get matrix B (~get feature vectors)
    // cout << "\nGetting B.\n";
    int n_theta_y = theta_y.size();
    int n_theta_yddot = theta_yddot.size();
    int n_theta = n_theta_y + n_theta_yddot;
    MatrixXd B = MatrixXd::Zero(A.rows(), n_theta);
    if (setting.cubic_spline_in_rom_constraint) {
      // Get the row index of B matrix where dynamics constraint starts
      VectorXd ind_head = solvers::GetConstraintRows(
          *gm_traj_opt.dircon,
          gm_traj_opt.dynamics_constraint_at_head_bindings[0]);
      // cout << "ind_head = " << ind_head(0) << endl;
      // VectorXd ind_tail = solvers::GetConstraintRows(
      //                     *gm_traj_opt.dircon.get(),
      //                     gm_traj_opt.dynamics_constraint_at_tail_bindings[0]);
      // cout << "ind_tail = " << ind_tail(0) << endl;
      int N_accum = 0;
      for (unsigned int l = 0; l < num_time_samples.size(); l++) {
        for (int m = 0; m < num_time_samples[l] - 1; m++) {
          int i = N_accum + m;
          // cout << "i = " << i << endl;
          // Get the gradient value first
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
          auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
          auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
          auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);
          VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
          VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
          VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

          MatrixXd dyn_gradient_head =
              gm_traj_opt.dynamics_constraint_at_head->getGradientWrtTheta(
                  x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);
          // MatrixXd dyn_gradient_tail =
          //   gm_traj_opt.dynamics_constraint_at_tail->getGradientWrtTheta(
          //     x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);

          // Fill in B matrix
          B.block(ind_head(0) + i * n_yddot, 0, n_yddot, n_theta) =
              dyn_gradient_head;
          // B.block(ind_tail(0) + i * 2 * n_yddot, 0, n_yddot, n_theta)
          //   = dyn_gradient_tail;
          // cout << "row " << ind_head(0) + i * 2 * n_yddot << endl;
          // cout << "row " << ind_tail(0) + i * 2 * n_yddot << endl << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    } else {
      // Get the row index of B matrix where dynamics constraint starts
      VectorXd ind_start = solvers::GetConstraintRows(
          *gm_traj_opt.dircon,
          gm_traj_opt.dynamics_constraint_at_knot_bindings[0]);
      // cout << "ind_start = " << ind_start(0) << endl;
      int N_accum = 0;
      for (unsigned int l = 0; l < num_time_samples.size(); l++) {
        for (int m = 0; m < num_time_samples[l]; m++) {
          int time_index = N_accum + m;
          // cout << "i = " << i << endl;
          // Get the gradient value first
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto u_i = gm_traj_opt.dircon->input(time_index);
          auto lambda_i = gm_traj_opt.dircon->force(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(time_index, n_tau);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd u_i_sol = result.GetSolution(u_i);
          VectorXd lambda_i_sol = result.GetSolution(lambda_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);

          MatrixXd dyn_gradient =
              gm_traj_opt.dynamics_constraint_at_knot[l]->getGradientWrtTheta(
                  x_i_sol, u_i_sol, lambda_i_sol, tau_i_sol);

          // Fill in B matrix
          B.block(ind_start(0) + time_index * n_yddot, 0, n_yddot, n_theta) =
              dyn_gradient;
          // cout << "row " << ind_start(0) + i * 2 * n_yddot << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Augment the constraint matrices and vectors (B, A, y, lb, ub)
    // so that we fix the scaling of the model parameters
    /*augmentConstraintToFixThetaScaling(B, A, y, lb, ub,
                                       n_y, n_feature_s, theta_y, sample_idx);*/

    // Store the vectors and matrices
    // cout << "\nStoring vectors and matrices into csv.\n";
    /*writeCSV(directory + prefix + string("H.csv"), H);
    writeCSV(directory + prefix + string("b.csv"), b);
    writeCSV(directory + prefix + string("A.csv"), A);
    writeCSV(directory + prefix + string("lb.csv"), lb);
    writeCSV(directory + prefix + string("ub.csv"), ub);
    writeCSV(directory + prefix + string("y.csv"), y);
    writeCSV(directory + prefix + string("B.csv"), B);*/

    //    auto start = std::chrono::high_resolution_clock::now();
    QPs.w_sol_vec[sample_idx]->resizeLike(w_sol);
    QPs.A_vec[sample_idx]->resizeLike(A);
    QPs.H_vec[sample_idx]->resizeLike(H);
    QPs.y_vec[sample_idx]->resizeLike(y);
    QPs.lb_vec[sample_idx]->resizeLike(lb);
    QPs.ub_vec[sample_idx]->resizeLike(ub);
    QPs.b_vec[sample_idx]->resizeLike(b);
    QPs.B_vec[sample_idx]->resizeLike(B);
    *(QPs.w_sol_vec[sample_idx]) = w_sol;
    *(QPs.A_vec[sample_idx]) = A;
    *(QPs.H_vec[sample_idx]) = H;
    *(QPs.y_vec[sample_idx]) = y;
    *(QPs.lb_vec[sample_idx]) = lb;
    *(QPs.ub_vec[sample_idx]) = ub;
    *(QPs.b_vec[sample_idx]) = b;
    *(QPs.B_vec[sample_idx]) = B;
    /*auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "time it takes to save matrices/vectors to RAM: " << elapsed.count()
         << endl;*/
    // It takes about 5 ms or 25 ms to store the above variables. (I guess 25ms
    // is when it needs to resize to bigger memory)

    // Store y, ydot, yddot and tau into csv files
    // cout << "\nStoring y, ydot and yddot into csv.\n";
    cout << "setting.cubic_spline_in_rom_constraint = "
         << setting.cubic_spline_in_rom_constraint << endl;
    if (setting.cubic_spline_in_rom_constraint) {
      std::vector<VectorXd> y_vec;
      std::vector<VectorXd> ydot_vec;
      std::vector<VectorXd> yddot_vec;
      std::vector<VectorXd> tau_vec;
      std::vector<VectorXd> h_vec;
      int N_accum = 0;
      // for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
      for (unsigned int l = 0; l < 1; l++) {  // just look at the first mode now
        for (int m = 0; m < num_time_samples[l]; m++) {
          int i = N_accum + m;
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);

          VectorXd y =
              gm_traj_opt.dynamics_constraint_at_head->GetY(x_i_sol.head(n_q));
          VectorXd ydot =
              gm_traj_opt.dynamics_constraint_at_head->GetYdot(x_i_sol);
          VectorXd yddot = gm_traj_opt.dynamics_constraint_at_head->GetYddot(
              y, ydot, tau_i_sol);
          y_vec.push_back(y);
          ydot_vec.push_back(ydot);
          yddot_vec.push_back(yddot);
          tau_vec.push_back(tau_i_sol);

          if (m < num_time_samples[l] - 1) {
            auto h_i = gm_traj_opt.dircon->timestep(i);
            VectorXd h_i_sol = result.GetSolution(h_i);
            h_vec.push_back(h_i_sol);
          }
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }

      // Save y, ydot and yddot
      // TODO: need to use CheckSplineOfY in case yddot doesn't match the spline
      //  constructed from y and ydot
      PiecewisePolynomial<double> s_spline =
          CreateCubicSplineGivenYAndYdot(h_vec, y_vec, ydot_vec);
      StoreSplineOfY(h_vec, s_spline, directory, prefix);
      storeTau(h_vec, tau_vec, directory, prefix);
      // CheckSplineOfY(h_vec, yddot_vec, s_spline);
    } else {
      // Only store the data of the first mode now
      MatrixXd t_and_y(1 + n_y, num_time_samples[0]);
      MatrixXd t_and_ydot(1 + n_y, num_time_samples[0]);
      MatrixXd t_and_yddot(1 + n_y, num_time_samples[0]);
      MatrixXd t_and_tau(1 + n_tau, num_time_samples[0]);
      t_and_y(0, 0) = 0;
      t_and_ydot(0, 0) = 0;
      t_and_yddot(0, 0) = 0;
      t_and_tau(0, 0) = 0;
      int N_accum = 0;
      for (unsigned int l = 0; l < 1; l++) {
        for (int m = 0; m < num_time_samples[l]; m++) {
          int i = N_accum + m;
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);

          VectorXd y = gm_traj_opt.dynamics_constraint_at_knot[l]->GetY(
              x_i_sol.head(n_q));
          VectorXd ydot =
              gm_traj_opt.dynamics_constraint_at_knot[l]->GetYdot(x_i_sol);
          VectorXd yddot = gm_traj_opt.dynamics_constraint_at_knot[l]->GetYddot(
              y, ydot, tau_i_sol);
          t_and_y.block(1, m, n_y, 1) = y;
          t_and_ydot.block(1, m, n_y, 1) = ydot;
          t_and_yddot.block(1, m, n_y, 1) = yddot;
          t_and_tau.block(1, m, n_tau, 1) = tau_i_sol;

          if (m < num_time_samples[l] - 1) {
            auto h_i = gm_traj_opt.dircon->timestep(i);
            VectorXd h_i_sol = result.GetSolution(h_i);
            t_and_y(0, m + 1) = t_and_y(0, m) + h_i_sol(0);
            t_and_ydot(0, m + 1) = t_and_ydot(0, m) + h_i_sol(0);
            t_and_yddot(0, m + 1) = t_and_yddot(0, m) + h_i_sol(0);
            t_and_tau(0, m + 1) = t_and_tau(0, m) + h_i_sol(0);
          }
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
      writeCSV(directory + prefix + string("t_and_y.csv"), t_and_y);
      writeCSV(directory + prefix + string("t_and_ydot.csv"), t_and_ydot);
      writeCSV(directory + prefix + string("t_and_yddot.csv"), t_and_yddot);
      writeCSV(directory + prefix + string("t_and_tau.csv"), t_and_tau);
      cout << "t_and_y = \n" << t_and_y << endl;
      cout << "t_and_ydot = \n" << t_and_ydot << endl;
      cout << "t_and_yddot = \n" << t_and_yddot << endl;
      cout << "t_and_tau = \n" << t_and_tau << endl;
    }

    // Below are all for debugging /////////////////////////////////////////////

    // Checking B
    // BTW, the code only work in the case of y = q_1 ^2 and yddot = y^3
    bool is_checking_matrix_B = false;
    int N_accum = 0;
    if (is_checking_matrix_B) {
      N_accum = 0;
      for (unsigned int l = 0; l < 1; l++) {  // just look at the first mode now
        for (int m = 0; m < num_time_samples[l] - 1; m++) {
          int i = N_accum + m;
          cout << "i = " << i << endl;

          // From finite differencing
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
          auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
          auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
          auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);
          VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
          VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
          VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);
          double h_i = h_i_sol(0);

          MatrixXd grad_head_byFD =
              gm_traj_opt.dynamics_constraint_at_head->getGradientWrtTheta(
                  x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);
          // MatrixXd grad_tail_byFD =
          //   gm_traj_opt.dynamics_constraint_at_tail->getGradientWrtTheta(
          //     x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);

          // From hand calculation (theta_y part)
          double phis_i = x_i_sol(1) * x_i_sol(1);
          double dphis_i = 2 * x_i_sol(1) * x_i_sol(1 + 7);
          double phis_iplus1 = x_iplus1_sol(1) * x_iplus1_sol(1);
          double dphis_iplus1 = 2 * x_iplus1_sol(1) * x_iplus1_sol(1 + 7);
          double grad_head_exact =
              (-6 * (phis_i - phis_iplus1) -
               2 * h_i * (2 * dphis_i + dphis_iplus1)) /
                  (h_i * h_i) -
              theta_yddot(0) * (3 * pow(theta_y(0), 2) * pow(phis_i, 3));
          // double grad_tail_exact =
          //   (6 * (phis_i - phis_iplus1) +
          //    2 * h_i * (dphis_i + 2 * dphis_iplus1)) / (h_i * h_i) -
          //   theta_yddot(0) * (3 * pow(theta_y(0), 2) * pow(phis_iplus1, 3));

          // From hand calculation (theta_yddot part)
          double dyn_feature_i = pow(theta_y(0) * phis_i, 3);
          // double dyn_feature_iplus1 = pow(theta_y(0) * phis_iplus1, 3);

          // Compare the values
          cout << grad_head_byFD << " (by finite difference)" << endl;
          cout << grad_head_exact << " " << -dyn_feature_i
               << " (analytically (exact solution))" << endl;
          cout << "  differnce = " << grad_head_byFD(0, 0) - grad_head_exact
               << ", " << grad_head_byFD(0, 1) + dyn_feature_i << endl;
          // cout << grad_tail_byFD << " (by finite difference)" << endl;
          // cout << grad_tail_exact << " " << -dyn_feature_iplus1 <<
          //      " (analytically (exact solution))" << endl;
          // cout << "  differnce = " << grad_tail_byFD(0, 0) - grad_tail_exact
          // <<
          //      ", " << grad_tail_byFD(0, 1) + dyn_feature_iplus1 << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Checking the accuracy of y and ydot calculation
    // BTW, the code only work in the case of y = q_1 ^2
    bool is_checking_y_ydot = false;
    if (is_checking_y_ydot) {
      N_accum = 0;
      for (unsigned int l = 0; l < 1; l++) {  // just look at the first mode now
        for (int m = 0; m < num_time_samples[l] - 1; m++) {
          // int i = N_accum + m;
          // cout << "i = " << i << endl;

          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);

          VectorXd y_i =
              gm_traj_opt.dynamics_constraint_at_head->GetY(x_i_sol.head(n_q));
          VectorXd ydot_i =
              gm_traj_opt.dynamics_constraint_at_head->GetYdot(x_i_sol);
          VectorXd y_iplus1 = gm_traj_opt.dynamics_constraint_at_head->GetY(
              x_iplus1_sol.head(n_q));
          VectorXd ydot_iplus1 =
              gm_traj_opt.dynamics_constraint_at_head->GetYdot(x_iplus1_sol);
          // cout << "ydot_i_byhand - ydot_i = " <<
          // theta_y(0) * 2 * x_i_sol(1)*x_i_sol(1 + 7) - ydot_i(0) << endl;
          // cout << "ydot_iplus1_byhand - ydot_iplus1 = " <<
          // theta_y(0) * 2 * x_iplus1_sol(1)*x_iplus1_sol(1 + 7) -
          // ydot_iplus1(0) << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // clang-format off

    bool is_comparing_two_constraint_linearization = false;
    if (is_comparing_two_constraint_linearization) {
      /*// Comparing the new cosntraint linearization and the old lienarization
      MatrixXd A_new;
      VectorXd y_new, lb_new, ub_new;
      systems::trajectory_optimization::newlinearizeConstraints(
        gm_traj_opt.dircon.get(), w_sol, y_new, A_new, lb_new, ub_new);
      // reorganize the rows or A
      cout << "Reorganize the rows or A\n";
      int nl_i = A.rows();
      int nw_i = A.cols();
      cout << "size of A = " << A.rows() << ", " <<  A.cols() << endl;
      cout << "size of A_new = " << A_new.rows() << ", " <<  A_new.cols() << endl;
      vector<int> new_row_idx;
      VectorXd rowi(nw_i);
      VectorXd rowj(nw_i);
      VectorXd normalized_rowi(nw_i);
      VectorXd normalized_rowj(nw_i);
      for (int i = 0; i < nl_i; i++) { // A_new
        for (int j = 0; j < nl_i; j++) { // A
          rowi = A_new.row(i).transpose();
          rowj = A.row(j).transpose();
          normalized_rowi = rowi / rowi.norm();
          normalized_rowj = rowj / rowj.norm();
          if ((normalized_rowi - normalized_rowj).norm() < 1e-15) {
            if (rowi.norm() != rowj.norm()) {
              cout << i << "-th row of A_new: scale are different by " <<
                   rowi.norm() / rowj.norm() << endl;
            }
            // check ub and lb
            // cout << "("<<i<<","<<j<<")"<<": \n";
            // cout << "  ub(j) = " << ub(j) << endl;
            // cout << "  ub_new(i) = " << ub_new(i) << endl;
            // cout << "  lb(j) = " << lb(j) << endl;
            // cout << "  lb_new(i) = " << lb_new(i) << endl;
            if (ub(j) == ub_new(i) && lb(j) == lb_new(i) ) {
              // Maybe there are duplicated rows, so check if already assigned the row
              bool is_existing = false;
              for (int idx : new_row_idx) {
                if (idx == j) {
                  is_existing = true;
                  break;
                }
              }
              if (!is_existing) {
                new_row_idx.push_back(j);
                break;
              }
            }
          }
          if (j == nl_i - 1) cout << i << "-th row of A_new has no corresponding A\n";
        }
      }
      MatrixXd A_new_reorg(nl_i, nw_i);
      for (int i = 0; i < nl_i; i++) {
        A_new_reorg.row(new_row_idx[i]) = A_new.row(i);
      }
      cout << "size of new_row_idx = " << new_row_idx.size() << endl;
      // Check if new_row_idx covers every index
      for (int i = 0; i < nl_i; i++) {
        for (int j = 0; j < nl_i; j++) {
          if (new_row_idx[j] == i) {
            break;
          }
          if (j == nl_i - 1 ) cout << i << "-th row of A doesn't get assigned to\n";
        }
      }
      // for (int i:new_row_idx) cout << i << endl;

      // compare A with A_new_reorg
      MatrixXd diff_A = A_new_reorg - A;
      MatrixXd abs_diff_A = diff_A.cwiseAbs();
      VectorXd left_one = VectorXd::Ones(abs_diff_A.rows());
      VectorXd right_one = VectorXd::Ones(abs_diff_A.cols());
      cout << "sum-abs-diff_A: " <<
           left_one.transpose()*abs_diff_A*right_one << endl;
      cout << "sum-abs-diff_A divide by m*n: " <<
           left_one.transpose()*abs_diff_A*right_one /
           (abs_diff_A.rows()*abs_diff_A.cols())
           << endl;
      double max_diff_A_element = abs_diff_A(0, 0);
      for (int i = 0; i < abs_diff_A.rows(); i++)
        for (int j = 0; j < abs_diff_A.cols(); j++) {
          if (abs_diff_A(i, j) > max_diff_A_element) {
            max_diff_A_element = abs_diff_A(i, j);
            cout << "(" << i << "," << j << ")" << ": max_diff_A_element = " <<
                 max_diff_A_element << endl;
          }
        }
      cout << "max_diff_A_element = " << max_diff_A_element << endl;*/
    }









    // int A_row = A.rows();
    // int A_col = A.cols();
    // cout << "A_row = " << A_row << endl;
    // cout << "A_col = " << A_col << endl;
    // int max_row_col = (A_row > A_col) ? A_row : A_col;
    // Eigen::BDCSVD<MatrixXd> svd_5(A);
    // cout << "A:\n";
    // cout << "  biggest singular value is " << svd_5.singularValues()(0) << endl;
    // cout << "  smallest singular value is "
    //      << svd_5.singularValues()(max_row_col - 1) << endl;





    /*cout << "\ncheck if H is diagonal: \n";
    MatrixXd H_test = H;
    int nw = H_test.rows();
    for (int i = 0; i < nw; i++) {
      H_test(i, i) = 0;
    }
    if (VectorXd::Ones(nw).transpose()*H_test * VectorXd::Ones(nw) == 0)
      cout << "H is diagonal" << endl;

    cout << "checking b\n";
    cout << "b.norm() = " << b.norm() << endl;

    cout << "norm of y = " << y.norm() << endl;





    // Get the index of the rows of active constraints
    vector<int> active_eq_row_idx;
    vector<int> ub_active_ineq_row_idx;
    vector<int> lb_active_ineq_row_idx;
    double tol = 1e-8; //1e-4
    for (int i = 0; i < y.rows(); i++) {
      if (ub(i) == lb(i))
        active_eq_row_idx.push_back(i);
      else if (y(i) >= ub(i) - tol)
        ub_active_ineq_row_idx.push_back(i);
      else if (y(i) <= lb(i) + tol)
        lb_active_ineq_row_idx.push_back(i);
    }
    unsigned int n_ae = active_eq_row_idx.size();
    unsigned int n_aub = ub_active_ineq_row_idx.size();
    unsigned int n_alb = lb_active_ineq_row_idx.size();
    cout << "n_ae = " << n_ae << endl;
    cout << "n_aub = " << n_aub << endl;
    cout << "n_alb = " << n_alb << endl;




    cout << "\nRun traj opt to check if your quadratic approximation is correct\n";
    int nl_i = A.rows();
    int nw_i = A.cols();
    MathematicalProgram quadprog;
    auto dw = quadprog.NewContinuousVariables(nw_i, "dw");
    quadprog.AddLinearConstraint( A,
                                  lb - y,
                                  ub - y,
                                  dw);
    quadprog.AddQuadraticCost(H, b, dw);
    // quadprog.SetSolverOption(drake::solvers::GurobiSolver::id(), "BarConvTol", 1E-9);
    quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Print file", "snopt.out");
    quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major iterations limit", 10000);
    // quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", 1.0e-14); //1.0e-10
    // quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor feasibility tolerance", 1.0e-14); //1.0e-10
    const auto result2 = Solve(quadprog);
    auto solution_result2 = result2.get_solution_result();
    cout << solution_result2 << endl;
    cout << "Cost:" << result2.get_optimal_cost() << endl;
    VectorXd dw_sol = result2.GetSolution(quadprog.decision_variables());
    cout << "dw_sol norm:" << dw_sol.norm() << endl;
    // cout << "dw_sol = \n" << dw_sol << endl;
    cout << "This should be zero\n" << VectorXd::Ones(nl_i).transpose()*A*dw_sol <<
         endl; // dw in null space
    cout << "if this is not zero, then w=0 is not optimal: " << dw_sol.transpose()*b
         << endl;
    cout << "Finished traj opt\n\n";

    // vector<double> w_sol_sort;
    // for(int i=0; i<dw_sol.size(); i++){
    //   w_sol_sort.push_back(dw_sol(i));
    // }
    // std::sort(w_sol_sort.begin(), w_sol_sort.end());
    // for(double w_sol_sort_ele : w_sol_sort)
    //   cout << w_sol_sort_ele << endl;

    // // Check if dw=0 violates any constraint
    // VectorXd QP_ub = ub-y;
    // VectorXd QP_lb = lb-y;
    // for(int i=0; i<nl_i; i++){
    //   if(QP_ub(i)<0)
    //     cout<< "row " << i << ": upper bound is smaller than 0 by " << QP_ub(i) << endl;
    //   if(QP_lb(i)>0)
    //     cout<< "row " << i << ": lower bound is larger than 0 by " << QP_lb(i) << endl;
    // }
    // cout << endl;
    // // Check if dw* to the QP violates any constraint
    // VectorXd QP_constraint_val = A*dw_sol;
    // for(int i=0; i<nl_i; i++){
    //   if(QP_ub(i) < QP_constraint_val(i))
    //     cout<< "row " << i << ": upper bound constraint violation by " << QP_constraint_val(i) - QP_ub(i) << endl;
    //   if(QP_lb(i) > QP_constraint_val(i))
    //     cout<< "row " << i << ": lower bound constraint violation by " << QP_constraint_val(i) - QP_lb(i) << endl;
    // }
    // cout << endl;




    // Plug back and check the cost and constraints of nonlinear programming
    double eps = 1e-1;
    unsigned int n_show = 10;  // number of rows of constraints you want to show
    // cost
    cout << "checking the cost of the original nonlinear programming and the approximated quadratic programming\n";
    for (int i = 0; i < 10 ; i++) {
      VectorXd w_sol_test = w_sol + i * eps * dw_sol;
      MatrixXd H2;
      VectorXd b2;
      double c_nonlinear;
      c_nonlinear = solvers::SecondOrderCost(
                      *gm_traj_opt.dircon.get(), w_sol_test, &H2, &b2) - c_double;
      cout << "i = " << i << endl;
      cout << "  c_nonlinear = " << c_nonlinear << endl;
      VectorXd dw_sol_test = i * eps * dw_sol;
      double c_aquadprog = 0.5 * dw_sol_test.transpose() * H * dw_sol_test + b.dot(
                             dw_sol_test) + c_double - c_double;
      cout << "  c_aquadprog = " << c_aquadprog << endl;
      cout << "  c_aquadprog - c_nonlinear = " << c_aquadprog - c_nonlinear << endl;
    }
    // constraint
    if (n_ae) {
      cout << "\nchecking the equality constraints of the original nonlinear programming and the approximated quadratic programming\n";
      // pick constraint violation row index
      std::vector<int> constraint_vio_row_idx;
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_ae; j++) {
          double violation = y2(active_eq_row_idx[j]) - ub(active_eq_row_idx[j]);
          if (abs(violation) > 1e-8) {
            constraint_vio_row_idx.push_back(j);
            k++;
            if (k == n_show)
              break;
          }
          if (i == n_show - 1 && j == n_ae - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        if (constraint_vio_row_idx.size() >= n_show)
          break;
        else if (i != 10 - 1)
          constraint_vio_row_idx.clear();
      }
      cout << "  Row index of violation = ";
      for (int j : constraint_vio_row_idx) {
        cout << j << ", ";
      }
      cout << endl;
      // evaluate the chosen rows
      for (unsigned int i = 0; i < 10 ; i++) {
        cout << "i = " << i << endl;
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        cout << "  nonlinear_constraint_val = ";
        for (int j : constraint_vio_row_idx) {
          double violation = y2(active_eq_row_idx[j]) - ub(active_eq_row_idx[j]);
          cout << violation << ", ";
        }
        cout << endl;
      }
    }
    if (n_aub) {
      cout << "\nchecking the inequality constraints (active upper bound) of the original nonlinear programming and the approximated quadratic programming\n";
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        VectorXd nonlinear_constraint_val = VectorXd::Zero(n_show);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_aub; j++) {
          double violation =
            y2(ub_active_ineq_row_idx[j]) - ub(ub_active_ineq_row_idx[j]);
          if (violation > 1e-8) {
            nonlinear_constraint_val(k) = violation;
            k++;
            if (k == n_show)
              break;
          }
          if (j == n_aub - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        cout << "  nonlinear_constraint_val = "
             << nonlinear_constraint_val.transpose() << endl;
      }
    }
    if (n_alb) {
      cout << "\nchecking the inequality constraints (active lower bound) of the original nonlinear programming and the approximated quadratic programming\n";
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        VectorXd nonlinear_constraint_val = VectorXd::Zero(n_show);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_alb; j++) {
          double violation =
            y2(lb_active_ineq_row_idx[j]) - lb(lb_active_ineq_row_idx[j]);
          if (violation < - 1e-8) {
            nonlinear_constraint_val(k) = violation;
            k++;
            if (k == n_show)
              break;
          }
          if (j == n_alb - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        cout << "  nonlinear_constraint_val = "
             << nonlinear_constraint_val.transpose() << endl;
      }
    }*/

    // clang-format on
  }  // end of if(!is_get_nominal)
}

void fiveLinkRobotTrajOpt(const MultibodyPlant<double>& plant,
                          const MultibodyPlant<AutoDiffXd>& plant_autoDiff,
                          const ReducedOrderModel& rom,
                          const InnerLoopSetting& setting, const Task& task,
                          const SubQpData& QPs, bool is_get_nominal,
                          bool extend_model, int sample_idx, int n_rerun,
                          double cost_threshold_for_update, int N_rerun,
                          int rom_option, int robot_option) {
  double stride_length = task.get("stride length");
  double ground_incline = task.get("ground incline");
  double walking_vel = task.get("velocity");
  double duration = stride_length / walking_vel;

  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> input_map = multibody::makeNameToActuatorsMap(plant);
  // for (auto const& element : pos_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : vel_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : input_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  //  int n_x = n_q + n_v;
  int n_u = plant.num_actuators();
  // std::cout<<"n_x = "<<n_x<<"\n";
  // std::cout<<"n_u = "<<n_u<<"\n";

  const Body<double>& left_lower_leg = plant.GetBodyByName("left_lower_leg");
  const Body<double>& right_lower_leg = plant.GetBodyByName("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;
  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
  auto leftFootConstraint = DirconPositionData<double>(plant, left_lower_leg,
                                                       pt, isXZ, ground_normal);
  auto rightFootConstraint = DirconPositionData<double>(
      plant, right_lower_leg, pt, isXZ, ground_normal);

  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(mu);

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rightConstraints);

  auto leftOptions = DirconOptions(ls_dataset.countConstraints(), plant);
  leftOptions.setConstraintRelative(0, true);
  auto rightOptions = DirconOptions(rs_dataset.countConstraints(), plant);
  rightOptions.setConstraintRelative(0, true);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  // Constraint scaling
  for (int i = 0; i < 2; i++) {
    // before adding rom constraint
    /*// Dynamic constraints
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 0, 4);
    options_list[i].setDynConstraintScaling(1.0 / 60.0, 5, 5);
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 6, 6);
    options_list[i].setDynConstraintScaling(1.0 / 60.0, 7, 7);
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 8, 8);
    options_list[i].setDynConstraintScaling(1.0 / 100.0, 9, 9);
    options_list[i].setDynConstraintScaling(1.0 / 1000.0, 10, 10);
    options_list[i].setDynConstraintScaling(1.0 / 300.0, 11, 11);
    options_list[i].setDynConstraintScaling(1.0 / 3000.0, 12, 12);
    options_list[i].setDynConstraintScaling(1.0 / 400.0, 13, 13);
    // Kinematic constraints
    options_list[i].setKinConstraintScaling(1.0 / 1000.0, 0, 0);
    options_list[i].setKinConstraintScaling(1.0 / 50.0, 1, 1);
    // Impact constraints
    options_list[i].setImpConstraintScaling(1.0 / 20.0, 0, 1);
    options_list[i].setImpConstraintScaling(1.0 / 5.0, 2, 2);
    options_list[i].setImpConstraintScaling(1.0 / 3.0, 3, 3);
    options_list[i].setImpConstraintScaling(1.0 / 5.0, 4, 4);
    options_list[i].setImpConstraintScaling(1.0 / 1.0, 5, 5);
    options_list[i].setImpConstraintScaling(1.0 / 3.0, 6, 6);*/

    // after adding rom constraint
    // Dynamic constraints
    /*options_list[i].setDynConstraintScaling({0, 1, 2, 3}, 1.0 / 40.0);
    options_list[i].setDynConstraintScaling({4, 5}, 1.0 / 60.0);
    options_list[i].setDynConstraintScaling(6, 1.0 / 200.0); // end of pos
    options_list[i].setDynConstraintScaling(7, 1.0 / 180.0);
    options_list[i].setDynConstraintScaling(8, 1.0 / 120.0);
    options_list[i].setDynConstraintScaling(9, 1.0 / 300.0);
    options_list[i].setDynConstraintScaling(10, 1.0 / 1000.0);
    options_list[i].setDynConstraintScaling(11, 1.0 / 1500.0);
    options_list[i].setDynConstraintScaling(12, 1.0 / 3000.0);
    options_list[i].setDynConstraintScaling(13, 1.0 / 2800.0);
    // Kinematic constraints
    options_list[i].setKinConstraintScaling(0, 1.0 / 1000.0);
    options_list[i].setKinConstraintScaling(1, 1.0 / 25.0);
    // Impact constraints
    options_list[i].setImpConstraintScaling({0, 1}, 1.0 / 20.0);
    options_list[i].setImpConstraintScaling(2, 1.0 / 5.0);
    options_list[i].setImpConstraintScaling(3, 1.0 / 3.0);
    options_list[i].setImpConstraintScaling(4, 1.8 / 5.0);
    options_list[i].setImpConstraintScaling(5, 1.8 / 1.0);
    options_list[i].setImpConstraintScaling(6, 1.8 / 3.0);*/
  }

  // Stated in the MultipleShooting class:
  // This class assumes that there are a fixed number (N) time steps/samples,
  // and that the trajectory is discretized into timesteps h (N-1 of these),
  // state x (N of these), and control input u (N of these).
  std::vector<int> num_time_samples;
  num_time_samples.push_back(setting.n_node);
  num_time_samples.push_back(1);
  std::vector<double> min_dt;
  min_dt.push_back(0);
  min_dt.push_back(0);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  int N = 0;
  for (int num_time_sample : num_time_samples) N += num_time_sample;
  N -= num_time_samples.size() - 1;  // Overlaps between modes
  // std::cout<<"N = "<<N<<"\n";

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&rs_dataset);

  auto trajopt = std::make_unique<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // You can comment this out to not put any constraint on the time
  // However, we need it now, since we add the running cost by hand
  trajopt->AddDurationBounds(duration, duration);

  if (setting.use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt->SetSolverOption(id, "tol", setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "dual_inf_tol", setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "constr_viol_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "compl_inf_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "max_iter", setting.max_iter);
    trajopt->SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt->SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    trajopt->SetSolverOption(id, "print_timing_statistics", "no");
    trajopt->SetSolverOption(id, "print_level", 0);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt->SetSolverOption(id, "acceptable_compl_inf_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "acceptable_constr_viol_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    trajopt->SetSolverOption(id, "acceptable_tol", 1e2);
    trajopt->SetSolverOption(id, "acceptable_iter", 5);
  } else {
    // Snopt settings
    // cout << "WARNING: you are printing snopt log.\n for Cassie";
    // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
    //                         "../snopt_sample#"+to_string(sample_idx)+".out");
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major iterations limit", setting.max_iter);
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Iterations limit", 100000);  // QP subproblems
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                             0);  // 0
    trajopt->SetSolverOption(
        drake::solvers::SnoptSolver::id(), "Scale option",
        setting.snopt_scaling
            ? 2
            : 0);  // snopt doc said try 2 if seeing snopta exit 40
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance",
                             setting.major_optimality_tol);
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance",
                             setting.major_feasibility_tol);
  }

  // Periodicity constraints
  auto x0 = trajopt->initial_state();
  // auto xf = trajopt->final_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);

  // trajopt->AddLinearConstraint(x0(pos_map.at("planar_z")) == xf(
  //                                pos_map.at("planar_z")));
  trajopt->AddLinearConstraint(x0(pos_map.at("planar_roty")) ==
                               xf(pos_map.at("planar_roty")));
  trajopt->AddLinearConstraint(x0(pos_map.at("left_hip_pin")) ==
                               xf(pos_map.at("right_hip_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("left_knee_pin")) ==
                               xf(pos_map.at("right_knee_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("right_hip_pin")) ==
                               xf(pos_map.at("left_hip_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("right_knee_pin")) ==
                               xf(pos_map.at("left_knee_pin")));

  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_xdot")) ==
                               xf(n_q + vel_map.at("planar_xdot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_zdot")) ==
                               xf(n_q + vel_map.at("planar_zdot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_rotydot")) ==
                               xf(n_q + vel_map.at("planar_rotydot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("left_hip_pindot")) ==
                               xf(n_q + vel_map.at("right_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("left_knee_pindot")) ==
                               xf(n_q + vel_map.at("right_knee_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("right_hip_pindot")) ==
                               xf(n_q + vel_map.at("left_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("right_knee_pindot")) ==
                               xf(n_q + vel_map.at("left_knee_pindot")));

  // u periodic constraint
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  trajopt->AddLinearConstraint(u0(input_map.at("left_hip_torque")) ==
                               uf(input_map.at("right_hip_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("right_hip_torque")) ==
                               uf(input_map.at("left_hip_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("left_knee_torque")) ==
                               uf(input_map.at("right_knee_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("right_knee_torque")) ==
                               uf(input_map.at("left_knee_torque")));

  // Knee joint limits
  auto x = trajopt->state();
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_knee_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_knee_pin")) <=
                                        M_PI / 2.0);

  // hip joint limits
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_hip_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_hip_pin")) <=
                                        M_PI / 2.0);

  // x-distance constraint constraints
  trajopt->AddLinearConstraint(x0(pos_map.at("planar_x")) == 0);
  trajopt->AddLinearConstraint(xf(pos_map.at("planar_x")) == stride_length);

  // make sure it's left stance
  trajopt->AddLinearConstraint(x0(pos_map.at("left_hip_pin")) <=
                               x0(pos_map.at("right_hip_pin")));

  // testing -- swing foot height constraint
  Vector3d z_hat(0, 0, 1);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(z_hat, ground_normal);
  Eigen::Matrix3d T_ground_incline = q.matrix().transpose();
  auto right_foot_constraint_z =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "right_lower_leg", pt, T_ground_incline.row(2),
          0 * VectorXd::Ones(1),
          std::numeric_limits<double>::infinity() * VectorXd::Ones(1));
  for (int index = 1; index < num_time_samples[0] - 1; index++) {
    auto x_i = trajopt->state(index);
    trajopt->AddConstraint(right_foot_constraint_z, x_i.head(n_q));
  }

  // Scale decision variable
  // time
  trajopt->ScaleTimeVariables(0.04);
  // state
  //  trajopt->ScaleStateVariables(10, n_q, n_q + n_v - 1);
  // input
  //  trajopt->ScaleInputVariables(10, 0, 3);
  // force
  std::vector<int> idx_list;
  idx_list.clear();
  for (int i = 0; i < ls_dataset.countConstraintsWithoutSkipping(); i++) {
    idx_list.push_back(i);
  }
  trajopt->ScaleForceVariables(0, idx_list, 50);
  idx_list.clear();
  for (int i = 0; i < rs_dataset.countConstraintsWithoutSkipping(); i++) {
    idx_list.push_back(i);
  }
  trajopt->ScaleForceVariables(1, idx_list, 50);
  // impulse
  //  trajopt->ScaleImpulseVariables(
  //      10, 0, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);  // 0.1
  // Constraint slack
  //  trajopt->ScaleKinConstraintSlackVariables(50, 0, 0, 5);
  //  trajopt->ScaleKinConstraintSlackVariables(500, 0, 6, 7);

  // Add cost
  MatrixXd R = setting.R_double * MatrixXd::Identity(n_u, n_u);
  MatrixXd Q = setting.Q_double * MatrixXd::Identity(n_v, n_v);
  // Don't use AddRunningCost, cause it makes cost Hessian to be indefinite
  // I'll fix the timestep and add cost myself
  /*auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R * u);
  trajopt->AddRunningCost(x.transpose()*Q * x);*/
  // Add running cost by hand (Trapezoidal integration):
  double timestep = duration / (N - 1);
  trajopt->AddQuadraticCost(Q * timestep / 2, VectorXd::Zero(n_v),
                            x0.tail(n_v));
  trajopt->AddQuadraticCost(R * timestep / 2, VectorXd::Zero(n_u), u0);
  for (int i = 1; i <= N - 2; i++) {
    auto ui = trajopt->input(i);
    auto xi = trajopt->state(i);
    trajopt->AddQuadraticCost(Q * timestep, VectorXd::Zero(n_v), xi.tail(n_v));
    trajopt->AddQuadraticCost(R * timestep, VectorXd::Zero(n_u), ui);
  }
  trajopt->AddQuadraticCost(Q * timestep / 2, VectorXd::Zero(n_v),
                            xf.tail(n_v));
  trajopt->AddQuadraticCost(R * timestep / 2, VectorXd::Zero(n_u), uf);

  // Zero impact at touchdown
  if (setting.is_zero_touchdown_impact)
    trajopt->AddLinearConstraint(MatrixXd::Ones(1, 1), VectorXd::Zero(1),
                                 VectorXd::Zero(1),
                                 trajopt->impulse_vars(0).tail(1));

  // Move the trajectory optmization problem into GoldilocksModelTrajOpt
  // where we add the constraints for reduced order model
  GoldilocksModelTrajOpt gm_traj_opt(
      rom, std::move(trajopt), plant, num_time_samples, dataset_list,
      is_get_nominal, setting, rom_option, robot_option, 1 /*temporary*/);

  addRegularization(is_get_nominal, setting.eps_reg, gm_traj_opt);

  // initial guess if the file exists
  if (!setting.init_file.empty()) {
    setInitialGuessFromFile(setting.directory, setting.init_file, gm_traj_opt);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    gm_traj_opt.dircon->SetInitialGuessForAllVariables(
        VectorXd::Random(gm_traj_opt.dircon->decision_variables().size()));
  }

  // Testing
  //  cout << "Choose the best solver: " <<
  //       drake::solvers::ChooseBestSolver(*(gm_traj_opt.dircon)).name() <<
  //       endl;

  drake::solvers::SolverId solver_id("");

  if (setting.use_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    //    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(*gm_traj_opt.dircon);
    //    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  // cout << "Solving DIRCON (based on MultipleShooting)\n";
  auto start = std::chrono::high_resolution_clock::now();
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(*gm_traj_opt.dircon, gm_traj_opt.dircon->initial_guess(),
                gm_traj_opt.dircon->solver_options(), &result);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  // Save trajectory to file
  if (true) {
    string file_name = "dircon_trajectory";
    DirconTrajectory saved_traj(
        plant, *gm_traj_opt.dircon, result, file_name,
        "Decision variables and state/input trajectories");
    saved_traj.WriteToFile(setting.directory + file_name);
    std::cout << "Wrote to file: " << setting.directory + file_name
              << std::endl;
  }

  bool is_print_for_debugging = false;
  VectorXd w_sol;
  extractResult(w_sol, gm_traj_opt, result, elapsed, num_time_samples, N, plant,
                plant_autoDiff, setting, rom, task, QPs, sample_idx, n_rerun,
                cost_threshold_for_update, N_rerun, dataset_list,
                is_print_for_debugging);
  postProcessing(w_sol, gm_traj_opt, result, num_time_samples, N, plant,
                 plant_autoDiff, setting, rom, QPs, is_get_nominal,
                 extend_model, sample_idx, n_rerun, cost_threshold_for_update,
                 N_rerun, rom_option, robot_option);
}

void cassieTrajOpt(const MultibodyPlant<double>& plant,
                   const MultibodyPlant<AutoDiffXd>& plant_autoDiff,
                   const ReducedOrderModel& rom,
                   const InnerLoopSetting& setting, const Task& task,
                   const SubQpData& QPs, bool is_get_nominal, bool extend_model,
                   int sample_idx, int n_rerun,
                   double cost_threshold_for_update, int N_rerun,
                   int rom_option, int robot_option) {
  double stride_length = task.get("stride length");
  double ground_incline = task.get("ground incline");
  double walking_vel = task.get("velocity");
  double turning_rate = task.get("turning rate");
  double duration = stride_length / walking_vel;

  double all_cost_scale = setting.all_cost_scale;

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Walking modes
  int walking_mode = 0;  // 0: instant change of support
  // 1: single double single
  // 2: heel to toe
  DRAKE_DEMAND(walking_mode == 0);
  // If you want to use walking_mode 1 or 2, then you should either
  // 1. finish multi-phase ROM, or
  // 2. write the code here so it takes care of multi-mode. (e.g. only have rom
  //    in the first phase, etc)

  // Cost on velocity and input
  double w_Q = setting.Q_double * all_cost_scale;
  double w_R = setting.R_double * all_cost_scale;
  // Cost on force (the final weight is w_lambda^2)
  double w_lambda = 1.0e-4 * sqrt(all_cost_scale);
  // Cost on difference over time
  double w_lambda_diff = 0.000001 * 0.1 * all_cost_scale;
  double w_v_diff = 0.01 * 5 * 0.1 * all_cost_scale;  // TODO
  double w_u_diff = 0.00001 * 0.1 * all_cost_scale;
  // Cost on position
  double w_q_hip_roll = 1 * 5 * 10 * all_cost_scale;
  double w_q_hip_yaw = 1 * 5 * all_cost_scale;
  double w_q_quat = 1 * 5 * 10 * all_cost_scale;
  // Additional cost on pelvis
  double w_Q_vy = w_Q * 1;  // avoid pelvis rocking in y
  double w_Q_vz = w_Q * 1;  // avoid pelvis rocking in z
  // Additional cost on swing toe
  double w_Q_swing_toe = w_Q * 10;  // avoid swing toe shaking
  double w_R_swing_toe = w_R * 1;   // avoid swing toe shaking
  // Testing -- cost on position difference (cost on v is not enough, because
  // the solver might exploit the integration scheme. If we only penalize
  // velocity at knots, then the solver will converge to small velocity at knots
  // but big acceleration at knots!)
  double w_q_diff = 1 * 5 * 0.1 * all_cost_scale;
  double w_q_diff_swing_toe = w_q_diff * 1;
  // Testing
  double w_v_diff_swing_leg = w_v_diff * 1;
  // Testing
  double w_joint_accel = 0.0001;  // The final weight is w_joint_accel * W_Q

  // Flags for constraints
  bool swing_foot_ground_clearance = false;
  bool swing_leg_collision_avoidance = false;
  bool periodic_quaternion = false;
  bool periodic_joint_pos = true;
  bool periodic_floating_base_vel = false;
  bool periodic_joint_vel = true;
  bool periodic_effort = false;
  bool ground_normal_force_margin = false;
  bool zero_com_height_vel = false;
  // Testing
  bool zero_com_height_vel_difference = false;
  bool zero_pelvis_height_vel = false;

  // Optional constraints
  // This seems to be important at higher walking speeds
  bool constrain_stance_leg_fourbar_force = false;

  // Temporary solution for get rid of redundant four bar constraint in the
  // second mode (since we have periodic constraints on state and input, it's
  // fine to just ignore the four bar constraint (position, velocity and
  // acceleration levels))
  bool four_bar_in_right_support = true;

  // Testing
  bool only_one_mode = false;
  if (only_one_mode) {
    periodic_joint_pos = false;
    periodic_joint_vel = false;
    periodic_effort = false;
  }
  bool one_mode_full_states_constraint_at_boundary = false;
  bool one_mode_full_positions_constraint_at_boundary = true;
  bool one_mode_base_x_constraint_at_boundary = false;

  // Testing
  bool add_cost_on_collocation_vel = false;
  bool add_joint_acceleration_cost = true;
  bool not_trapo_integration_cost = false;
  bool add_joint_acceleration_constraint = false;
  double joint_accel_lb = -10;
  double joint_accel_ub = 10;
  bool add_hip_roll_pos_constraint = false;
  double hip_roll_pos_lb = -0.1;
  double hip_roll_pos_ub = 0.1;
  bool add_base_vy_constraint = false;
  double base_vy_lb = -0.4;
  double base_vy_ub = 0.4;

  // Testing
  bool lower_bound_on_ground_reaction_force_at_the_first_and_last_knot = false;

  // Testing
  bool pre_and_post_impact_efforts = true;
  if (pre_and_post_impact_efforts) {
    cout << "WARNING: you also need to set the flat "
            "`pre_and_post_impact_efforts` in HybridDricon to true.\n";
  }
  // TODO(yminchen): add dircon traj logging for post impact input

  // TODO(yminchen):
  //  reminder: if it solves very slowly, you might want to scale constraint
  //  because you added reflected inertia

  // Testing -- remove ankle joint pos/vel periodicity constraint becasue it's
  // redundant (because we have fourbar constraint at pos/vel level)
  bool remove_ankle_joint_from_periodicity = true;
  if (remove_ankle_joint_from_periodicity) {
    DRAKE_DEMAND(four_bar_in_right_support);
  }

  // Testing -- add epsilon to periodicity contraint in case we are over
  // cosntraining the problem
  bool relax_vel_periodicity_constraint = true;
  double eps_vel_period = 0.1;
  if (!relax_vel_periodicity_constraint) {
    eps_vel_period = 0;
  }
  bool relax_pos_periodicity_constraint = true;
  double eps_pos_period = 0.01;
  if (!relax_pos_periodicity_constraint) {
    eps_pos_period = 0;
  }

  // Setup cost matrices
  MatrixXd W_Q = w_Q * MatrixXd::Identity(n_v, n_v);
  W_Q(4, 4) = w_Q_vy;
  W_Q(5, 5) = w_Q_vz;
  W_Q(n_v - 1, n_v - 1) = w_Q_swing_toe;
  MatrixXd W_R = w_R * MatrixXd::Identity(n_u, n_u);
  W_R(n_u - 1, n_u - 1) = w_R_swing_toe;

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);
  //  for (auto mem : vel_map) {
  //    cout << mem.first << ", " << mem.second << endl;
  //  }

  // int n_x = n_q + n_v;
  // cout << "n_q = " << n_q << "\n";
  // cout << "n_v = " << n_v << "\n";
  // cout << "n_x = " << n_x << "\n";
  // cout << "n_u = " << n_u << "\n";
  // cout << "floating_positions_start = " <<
  //      plant.GetBodyByName("pelvis").floating_positions_start() << endl;
  // cout << "floating_velocities_start = " <<
  //      plant.GetBodyByName("pelvis").floating_velocities_start() << endl;

  // create joint/motor names
  vector<std::pair<string, string>> l_r_pairs{
      std::pair<string, string>("_left", "_right"),
      std::pair<string, string>("_right", "_left")};
  vector<string> asy_joint_names;
  vector<string> sym_joint_names;
  if (turning_rate == 0) {
    asy_joint_names = {"hip_roll", "hip_yaw"};
    if (remove_ankle_joint_from_periodicity) {
      sym_joint_names = {"hip_pitch", "knee", "toe"};
    } else {
      sym_joint_names = {"hip_pitch", "knee", "ankle_joint", "toe"};
    }
  } else {
    asy_joint_names = {"hip_roll"};
    sym_joint_names = {"hip_pitch"};
  }
  vector<string> joint_names{};
  vector<string> motor_names{};
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& asy_joint_name : asy_joint_names) {
      joint_names.push_back(asy_joint_name + l_r_pair.first);
      motor_names.push_back(asy_joint_name + l_r_pair.first + "_motor");
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      joint_names.push_back(sym_joint_names[i] + l_r_pair.first);
      if (sym_joint_names[i] != "ankle_joint") {
        motor_names.push_back(sym_joint_names[i] + l_r_pair.first + "_motor");
      }
    }
  }

  // Create ground normal for the problem
  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));

  // Set up contact/distance constraints
  const Body<double>& toe_left = plant.GetBodyByName("toe_left");
  const Body<double>& toe_right = plant.GetBodyByName("toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  bool isXZ = false;
  auto left_toe_front_constraint = DirconPositionData<double>(
      plant, toe_left, pt_front_contact, isXZ, ground_normal);
  auto left_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_left, pt_rear_contact, isXZ, ground_normal);
  auto right_toe_front_constraint = DirconPositionData<double>(
      plant, toe_right, pt_front_contact, isXZ, ground_normal);
  auto right_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_right, pt_rear_contact, isXZ, ground_normal);
  double mu = 1;
  left_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  left_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);

  const auto& thigh_left = plant.GetBodyByName("thigh_left");
  const auto& heel_spring_left = plant.GetBodyByName("heel_spring_left");
  const auto& thigh_right = plant.GetBodyByName("thigh_right");
  const auto& heel_spring_right = plant.GetBodyByName("heel_spring_right");
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);
  auto distance_constraint_left = DirconDistanceData<double>(
      plant, thigh_left, pt_on_thigh_left, heel_spring_left, pt_on_heel_spring,
      rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(
      plant, thigh_right, pt_on_thigh_right, heel_spring_right,
      pt_on_heel_spring, rod_length);

  // get rid of redundant constraint
  vector<int> skip_constraint_inds;
  skip_constraint_inds.push_back(3);

  // Left support
  vector<DirconKinematicData<double>*> ls_constraint;
  ls_constraint.push_back(&left_toe_front_constraint);
  ls_constraint.push_back(&left_toe_rear_constraint);
  ls_constraint.push_back(&distance_constraint_left);
  ls_constraint.push_back(&distance_constraint_right);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &ls_constraint,
                                                   skip_constraint_inds);
  // Right support
  vector<DirconKinematicData<double>*> rs_constraint;
  rs_constraint.push_back(&right_toe_front_constraint);
  rs_constraint.push_back(&right_toe_rear_constraint);
  if (four_bar_in_right_support) {
    rs_constraint.push_back(&distance_constraint_left);
    rs_constraint.push_back(&distance_constraint_right);
  }
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rs_constraint,
                                                   skip_constraint_inds);

  // Set up options
  std::vector<DirconOptions> options_list;
  options_list.push_back(DirconOptions(ls_dataset.countConstraints(), plant));
  if (!only_one_mode) {
    options_list.push_back(DirconOptions(rs_dataset.countConstraints(), plant));
  }

  // set force cost weight
  for (int i = 0; i < options_list.size(); i++) {
    options_list[i].setForceCost(w_lambda);
  }

  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf/rf | lr/rr | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7
  /// After skipping  || 0 1 2 |   3 4 | 5 6
  for (int i = 0; i < options_list.size(); i++) {
    options_list[i].setConstraintRelative(0, true);
    options_list[i].setConstraintRelative(1, true);
    options_list[i].setConstraintRelative(3, true);
  }
  // Constraint scaling
  double s = 1;  // 100;  // scale every nonlinear constraint together except
                 // quaternion
  for (int i = 0; i < options_list.size(); i++) {
    // options_list[i].setQuatConstraintScaling(s);
    if (is_get_nominal) {
      // old constraint scaling (from traj opt of cassie, without rom
      // constraint) Dynamic constraints
      options_list[i].setDynConstraintScaling({0, 1, 2, 3}, s / 30.0);
      options_list[i].setDynConstraintScaling(
          {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}, s / 60.0);
      options_list[i].setDynConstraintScaling({17, 18}, s / 300.0);
      options_list[i].setDynConstraintScaling(
          {19, 20, 21, 22, 23, 24, 25, 26, 27, 28}, s / 600.0);
      options_list[i].setDynConstraintScaling({29, 30, 31, 32, 33, 34},
                                              s / 3000.0);
      options_list[i].setDynConstraintScaling({35, 36}, s / 60000.0);
      // Kinematic constraints
      int n_l = options_list[i].getNumConstraints();
      options_list[i].setKinConstraintScaling({0, 1, 2, 3, 4}, s / 6000.0);
      options_list[i].setKinConstraintScaling(
          {n_l + 0, n_l + 1, n_l + 2, n_l + 3, n_l + 4}, s / 10.0);
      options_list[i].setKinConstraintScaling(
          {2 * n_l + 0, 2 * n_l + 1, 2 * n_l + 2, 2 * n_l + 3, 2 * n_l + 4}, s);
      if (i == 0 || four_bar_in_right_support) {
        options_list[i].setKinConstraintScaling({5, 6}, s / 600.0 * 2);
        options_list[i].setKinConstraintScaling({n_l + 5, n_l + 6}, s);
        options_list[i].setKinConstraintScaling({2 * n_l + 5, 2 * n_l + 6},
                                                s * 20);
      }
      // Impact constraints
      options_list[i].setImpConstraintScaling({0, 1, 2}, s / 50.0);
      options_list[i].setImpConstraintScaling({3, 4, 5}, s / 300.0);
      options_list[i].setImpConstraintScaling({6, 7}, s / 24.0);
      options_list[i].setImpConstraintScaling({8, 9}, s / 6.0);
      options_list[i].setImpConstraintScaling({10, 11, 12, 13}, s / 12.0);
      options_list[i].setImpConstraintScaling({14, 15}, s / 2.0);
      options_list[i].setImpConstraintScaling({16, 17}, s);
    } else {
      // new constraint scaling (20200227; after adding rom constraint)
      // Dynamic constraints
      options_list[i].setDynConstraintScaling({0, 1, 2, 3}, s * 1.0 / 30.0);
      options_list[i].setDynConstraintScaling(
          {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}, s * 1.0 / 60.0);
      options_list[i].setDynConstraintScaling({17, 18}, s * 1.0 / 300.0);
      options_list[i].setDynConstraintScaling({19, 20, 21, 22, 23, 24, 25, 26},
                                              s * 1.0 / 600.0);
      options_list[i].setDynConstraintScaling({27, 28, 29, 30, 31, 32, 33, 34},
                                              s * 1.0 / 3000.0);
      options_list[i].setDynConstraintScaling({35, 36}, s * 1.0 / 60000.0);
      // Kinematic constraints
      int n_l = options_list[i].getNumConstraints();
      options_list[i].setKinConstraintScaling({0, 1, 2, 3, 4}, s / 600.0);
      options_list[i].setKinConstraintScaling(
          {n_l + 0, n_l + 1, n_l + 2, n_l + 3, n_l + 4}, s / 1.0);
      options_list[i].setKinConstraintScaling(
          {2 * n_l + 0, 2 * n_l + 1, 2 * n_l + 2, 2 * n_l + 3, 2 * n_l + 4}, s);
      if (i == 0 || four_bar_in_right_support) {
        options_list[i].setKinConstraintScaling({5, 6}, s / 60.0 * 2);
        options_list[i].setKinConstraintScaling({n_l + 5, n_l + 6}, s * 10);
        options_list[i].setKinConstraintScaling({2 * n_l + 5, 2 * n_l + 6},
                                                s * 20);
      }
      // Impact constraints
      options_list[i].setImpConstraintScaling({0, 1, 2}, s / 5.0);
      options_list[i].setImpConstraintScaling({3, 4, 5}, s / 30.0);
      options_list[i].setImpConstraintScaling({6, 7}, s * 10.0 / 24.0);
      options_list[i].setImpConstraintScaling({8, 9}, s * 10.0 / 6.0);
      options_list[i].setImpConstraintScaling(10, s * 10.0 / 12.0);
      options_list[i].setImpConstraintScaling(11, s / 12.0);
      options_list[i].setImpConstraintScaling(12, s * 10.0 / 12.0);
      options_list[i].setImpConstraintScaling(13, s / 12.0);
      options_list[i].setImpConstraintScaling(14, s * 10.0 / 2.0);
      options_list[i].setImpConstraintScaling(15, s / 2.0);
      options_list[i].setImpConstraintScaling(16, s * 50.0);
      options_list[i].setImpConstraintScaling(17, s);
    }
  }

  // Testing -- TODO: delete this after done testing
  //  if (options_list.size() == 2) {
  //    options_list[1].setStartType(DirconKinConstraintType::kAccelOnly);
  //  }

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  vector<int> num_time_samples;
  vector<DirconKinematicDataSet<double>*> dataset_list;
  min_dt.push_back(0);
  max_dt.push_back(.3);
  num_time_samples.push_back(setting.n_node);
  dataset_list.push_back(&ls_dataset);
  if (!only_one_mode) {
    min_dt.push_back(0);
    max_dt.push_back(.3);
    num_time_samples.push_back(1);
    dataset_list.push_back(&rs_dataset);
  }

  auto trajopt = std::make_unique<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);

  if (setting.use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt->SetSolverOption(id, "tol", setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "dual_inf_tol", setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "constr_viol_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "compl_inf_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "max_iter", setting.max_iter);
    trajopt->SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt->SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    trajopt->SetSolverOption(id, "print_timing_statistics", "no");
    trajopt->SetSolverOption(id, "print_level", 0);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt->SetSolverOption(id, "acceptable_compl_inf_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "acceptable_constr_viol_tol",
                             setting.major_feasibility_tol);
    trajopt->SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    trajopt->SetSolverOption(id, "acceptable_tol", 1e2);
    trajopt->SetSolverOption(id, "acceptable_iter", 5);
  } else {
    //     Snopt settings
    cout << "WARNING: you are printing snopt log for Cassie.\n";
    trajopt->SetSolverOption(
        drake::solvers::SnoptSolver::id(), "Print file",
        "../snopt_sample#" + to_string(sample_idx) + ".out");
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major iterations limit", setting.max_iter);
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Iterations limit", 100000);  // QP subproblems
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                             0);  // 0
    trajopt->SetSolverOption(
        drake::solvers::SnoptSolver::id(), "Scale option",
        setting.snopt_scaling
            ? 2
            : 0);  // snopt doc said try 2 if seeing snopta exit 40
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance",
                             setting.major_optimality_tol);
    trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance",
                             setting.major_feasibility_tol);
  }

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // because of overlaps between modes

  // Get the decision variables that will be used
  auto u = trajopt->input();
  auto x = trajopt->state();
  auto u0 = trajopt->input(0);
  auto uf = (only_one_mode || !pre_and_post_impact_efforts)
                ? trajopt->input(N - 1)
                : trajopt->input_vars_by_mode(
                      num_time_samples.size() - 1,
                      num_time_samples[num_time_samples.size() - 1] - 1);
  auto x0 = trajopt->initial_state();
  auto xf = only_one_mode
                ? trajopt->final_state()
                : trajopt->state_vars_by_mode(
                      num_time_samples.size() - 1,
                      num_time_samples[num_time_samples.size() - 1] - 1);
  auto x_mid = trajopt->state(int(num_time_samples[0] / 2));

  // Testing
  if (only_one_mode) {
    VectorXd x0_val(plant.num_positions() + plant.num_velocities());
    VectorXd xf_val(plant.num_positions() + plant.num_velocities());
    // The following states come from traj opt with pos/vel periodicity
    // constraints and without ROM cosntraint.
    // Note that the velocities don't strictly mirror because the values for
    // post and pre impact vel are different!
    //    x0_val << 1, 0, 0, 0, 0, -0.00527375, 1.10066, 0.00686375, 0.0025628,
    //        0.000573004, -0.000573019, 0.439262, 0.201581, -0.646, -0.795755,
    //        0.866859, 1.01813, -1.53361, -1.29744, -0.321, 0.198875,
    //        -0.0342067, 0.67034, 0.363396, -0.0367492, -0.0100005, 0.0120047,
    //        0.0355722, 0.0585609, 0.26338, -0.584013, -1.45501,
    //        0.221981, 1.47458, -0.223655, -0.0841209, 0.797712;
    //    xf_val << 1, 0, 0, 0, 0.3, 0.00527375, 1.10066, -0.0025628,
    //    -0.00686375,
    //        0.000573019, -0.000573004, 0.201581, 0.439262, -0.795755, -0.646,
    //        1.01813, 0.866859, -1.29744, -1.53361, 0.49743, 0.0737923,
    //        0.0867892, 0.666384, -0.319709, -0.072308, -0.202807, 0.193892,
    //        -0.0869775, 0.763641, -0.705892, -0.590682, 0.214898, 1.02658,
    //        -0.216519, -1.04038, 0.780914, -0.269512;

    // This solution is derived with reflected inertia and pos/vel periodicity
    // constraint of two mode walking
    // From:
    // 20200926 try to impose lipm constraint/29 compare 1-mode traj with 2-mode
    // traj/Cost setting 1
    //    x0_val << 1, 0, 0, 0, 0, 0.00135545, 1.09939, -0.0768261, 0.0651368,
    //        0.0339166, -0.0339448, 0.431728, 0.1889, -0.646, -0.785052,
    //        0.866859, 1.00734, -1.52347, -1.28247, 1.68172E-08,
    //        -2.93932E-08, 1.30851E-11, 0.618169, 0.135654, -0.0738718,
    //        -0.10875, -0.110773, -0.000283673, 0.0345027, 0.0339327,
    //        -0.597203, -1.3188, -0.151959, 1.33654, 0.153149, -0.0479795,
    //        0.564141;
    //    xf_val << 1, 0, 0, 0, 0.3, -0.00135545, 1.09939, -0.0651368,
    //    0.0768261,
    //        0.0339448, -0.0339166, 0.1889, 0.431728, -0.785052,
    //        -0.646, 1.00734, 0.866859, -1.28247, -1.52347, 0.542888,
    //        -0.274931, 0.00598288, 0.601503, -0.146041, -0.140087, -0.387284,
    //        0.0312489, 0.012249, 0.130986, -0.824682, 0.305998, -0.225036,
    //        0.293566, 0.226799, -0.297514, 0.543059, -0.727215;

    // This solution is derived with reflected inertia and pos/vel periodicity
    // constraint of two mode walking
    // From:
    // 20200926 try to impose lipm constraint/29 compare 1-mode traj with 2-mode
    // traj/Cost setting 2/1 rederived start and end state from nomial traj/
    x0_val << 1, 0, 0, 0, 0, 0.00403739, 1.09669, -0.0109186, -0.00428689,
        0.0286625, -0.0286639, 0.455519, 0.229628, -0.646, -0.82116, 0.866859,
        1.04372, -1.54955, -1.3258, 1.95188E-07, -8.5509E-08, -8.03449E-11,
        0.660358, 0.322854, -0.0609757, -0.277685, -0.274869, -8.67676E-05,
        0.0667662, 0.0559658, -0.659971, -1.44404, -0.10276, 1.46346, 0.10347,
        -0.0674264, 0.631613;
    xf_val << 1, 0, 0, 0, 0.3, -0.00403739, 1.09669, 0.00428689, 0.0109186,
        0.0286639, -0.0286625, 0.229628, 0.455519, -0.82116, -0.646, 1.04372,
        0.866859, -1.3258, -1.54955, 0.51891, -0.174452, 0.100391, 0.661351,
        -0.338064, -0.12571, -0.183991, 0.218535, -0.101178, -0.0317852,
        -0.8024, -0.278951, -0.148364, 0.375203, 0.149389, -0.380249, 0.617827,
        -0.712411;

    // Set constraints for the first and last knot points
    if (one_mode_full_states_constraint_at_boundary) {
      trajopt->AddBoundingBoxConstraint(x0_val, x0_val, x0);
      trajopt->AddBoundingBoxConstraint(xf_val, xf_val, xf);
    } else if (one_mode_full_positions_constraint_at_boundary) {
      trajopt->AddBoundingBoxConstraint(x0_val.head(n_q), x0_val.head(n_q),
                                        x0.head(n_q));
      trajopt->AddBoundingBoxConstraint(xf_val.head(n_q), xf_val.head(n_q),
                                        xf.head(n_q));
    } else if (one_mode_base_x_constraint_at_boundary) {
      trajopt->AddBoundingBoxConstraint(x0_val.segment<1>(4),
                                        x0_val.segment<1>(4), x0.segment<1>(4));
      trajopt->AddBoundingBoxConstraint(xf_val.segment<1>(4),
                                        xf_val.segment<1>(4), xf.segment<1>(4));
    }
    //    trajopt->AddBoundingBoxConstraint(xf_val.head(7), xf_val.head(7),
    //                                      xf.head(7));
  }

  // Testing -- add start/end position constraint (TODO: delete this)
  VectorXd x0_val(plant.num_positions() + plant.num_velocities());
  VectorXd xf_val(plant.num_positions() + plant.num_velocities());
  // From:
  // 20200926 try to impose lipm constraint/29 compare 1-mode traj with 2-mode
  //  x0_val << 1, 0, 0, 0, 0, 0.00135545, 1.09939, -0.0768261, 0.0651368,
  //      0.0339166, -0.0339448, 0.431728, 0.1889, -0.646, -0.785052, 0.866859,
  //      1.00734, -1.52347, -1.28247, 1.68172E-08, -2.93932E-08, 1.30851E-11,
  //      0.618169, 0.135654, -0.0738718, -0.10875, -0.110773, -0.000283673,
  //      0.0345027, 0.0339327, -0.597203, -1.3188, -0.151959, 1.33654,
  //      0.153149, -0.0479795, 0.564141;
  //  xf_val << 1, 0, 0, 0, 0.3, -0.00135545, 1.09939, -0.0651368, 0.0768261,
  //      0.0339448, -0.0339166, 0.1889, 0.431728, -0.785052, -0.646, 1.00734,
  //      0.866859, -1.28247, -1.52347, 0.542888, -0.274931, 0.00598288,
  //      0.601503, -0.146041, -0.140087, -0.387284, 0.0312489, 0.012249,
  //      0.130986, -0.824682, 0.305998, -0.225036, 0.293566, 0.226799,
  //      -0.297514, 0.543059, -0.727215;
  // From:
  // 20200926 try to impose lipm constraint/29 compare 1-mode traj with 2-mode
  // traj/Cost setting 2/1 rederived start and end state from nomial traj/
  //  x0_val << 1, 0, 0, 0, 0, 0.00403739, 1.09669, -0.0109186, -0.00428689,
  //      0.0286625, -0.0286639, 0.455519, 0.229628, -0.646, -0.82116, 0.866859,
  //      1.04372, -1.54955, -1.3258, 1.95188E-07, -8.5509E-08, -8.03449E-11,
  //      0.660358, 0.322854, -0.0609757, -0.277685, -0.274869, -8.67676E-05,
  //      0.0667662, 0.0559658, -0.659971, -1.44404, -0.10276, 1.46346, 0.10347,
  //      -0.0674264, 0.631613;
  //  xf_val << 1, 0, 0, 0, 0.3, -0.00403739, 1.09669, 0.00428689, 0.0109186,
  //      0.0286639, -0.0286625, 0.229628, 0.455519, -0.82116, -0.646, 1.04372,
  //      0.866859, -1.3258, -1.54955, 0.51891, -0.174452, 0.100391, 0.661351,
  //      -0.338064, -0.12571, -0.183991, 0.218535, -0.101178, -0.0317852,
  //      -0.8024, -0.278951, -0.148364, 0.375203, 0.149389, -0.380249,
  //      0.617827, -0.712411;
  // From:
  // /home/yuming/Desktop/20200926 try to impose lipm constraint/33 play with
  // periodicity cosntraint/5 relax vel periodicity constraint/robot_1
  //  x0_val << 1, 0, 0, 0, 0, -0.00562789, 1.11289, 0.000132934, -0.00535692,
  //      0.0568366, -0.0568358, 0.379726, 0.0957978, -0.646, -0.696699,
  //      0.866859, 0.918171, -1.47408, -1.19106, 0.179286, 0.180633, -0.233861,
  //      0.620297, 0.366947, 0.000266116, -0.49626, -0.344546, 0.233888,
  //      0.237652, -0.205129, -0.605529, -0.538989, 0.482027, 0.546237,
  //      -0.487284, 0.396197, 0.755361;
  //  xf_val << 1, 0, 0, 0, 0.3, 0.00562789, 1.11289, 0.00535692, -0.000132934,
  //      0.0568358, -0.0568366, 0.0957978, 0.379726, -0.696699, -0.646,
  //      0.918171, 0.866859, -1.19106, -1.47408, 0.131213, 0.0349848, 0.289911,
  //      0.631216, -0.315915, -0.000545674, 0.193521, 0.00283896, -0.289819,
  //      -0.246365, -0.822828, -0.0944468, 0.546554, 0.314669, -0.552515,
  //      -0.318901, 0.84682, -0.888837;
  //  auto xf_end_of_first_mode = trajopt->final_state();
  //  trajopt->AddBoundingBoxConstraint(x0_val.head(n_q), x0_val.head(n_q),
  //                                    x0.head(n_q));
  //  trajopt->AddBoundingBoxConstraint(xf_val.head(n_q), xf_val.head(n_q),
  //                                    xf_end_of_first_mode.head(n_q));
  //  trajopt->AddBoundingBoxConstraint(x0_val.head(6), x0_val.head(6),
  //  x0.head(6)); trajopt->AddBoundingBoxConstraint(xf_val.head(6),
  //  xf_val.head(6),
  //                                    xf_end_of_first_mode.head(6));
  //  trajopt->AddBoundingBoxConstraint(x0_val.segment(7, n_q - 7),
  //                                    x0_val.segment(7, n_q - 7),
  //                                    x0.segment(7, n_q - 7));
  //  trajopt->AddBoundingBoxConstraint(xf_val.segment(7, n_q - 7),
  //                                    xf_val.segment(7, n_q - 7),
  //                                    xf_end_of_first_mode.segment(7, n_q -
  //                                    7));

  // Fix time duration
  trajopt->AddDurationBounds(duration, duration);

  double turning_angle = turning_rate * duration;
  if (!periodic_quaternion && !only_one_mode) {
    // Constraint on INITIAL floating base quaternion
    trajopt->AddBoundingBoxConstraint(1, 1, x0(pos_map.at("base_qw")));
    trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_qx")));
    trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_qy")));
    trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_qz")));

    // Constraint on FINAL floating base quaternion
    // TODO: below is a naive version. You can implement the constraint using
    //  rotation matrix and mirror around the x-z plane of local frame (however,
    //  the downside is the potential complexity of constraint)
    trajopt->AddBoundingBoxConstraint(cos(turning_angle / 2),
                                      cos(turning_angle / 2),
                                      xf(pos_map.at("base_qw")));
    trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_qx")));
    trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_qy")));
    trajopt->AddBoundingBoxConstraint(sin(turning_angle / 2),
                                      sin(turning_angle / 2),
                                      xf(pos_map.at("base_qz")));
  }

  // other floating base constraints
  if (!only_one_mode) {
    if (turning_rate == 0) {
      // x position constraint
      trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
      trajopt->AddBoundingBoxConstraint(stride_length, stride_length,
                                        xf(pos_map.at("base_x")));

      // Floating base periodicity
      trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) ==
                                   -xf(pos_map.at("base_y")));
      if (periodic_quaternion) {
        trajopt->AddLinearConstraint(x0(pos_map.at("base_qw")) ==
                                     xf(pos_map.at("base_qw")));
        trajopt->AddLinearConstraint(x0(pos_map.at("base_qx")) ==
                                     -xf(pos_map.at("base_qx")));
        trajopt->AddLinearConstraint(x0(pos_map.at("base_qy")) ==
                                     xf(pos_map.at("base_qy")));
        trajopt->AddLinearConstraint(x0(pos_map.at("base_qz")) ==
                                     -xf(pos_map.at("base_qz")));
      }
      if (periodic_floating_base_vel) {
        // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
        //                             xf(n_q + vel_map.at("base_wx")));
        // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
        //                             -xf(n_q + vel_map.at("base_wy")));
        // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
        //                             xf(n_q + vel_map.at("base_wz")));
        // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
        //                             xf(n_q + vel_map.at("base_vx")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                                     -xf(n_q + vel_map.at("base_vy")));
        // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
        //                             xf(n_q + vel_map.at("base_vz")));
      }
    } else {
      DRAKE_DEMAND(false);  // need to update this block of code first

      // z position constraint
      // We don't need to impose this constraint when turning rate is 0, because
      // the periodicity constraint already enforce the z height implicitly
      trajopt->AddLinearConstraint(x0(pos_map.at("base_z")) ==
                                   xf(pos_map.at("base_z")) +
                                       xf(pos_map.at("base_x")) *
                                           sin(ground_incline));

      // x y position constraint
      double radius = stride_length / abs(turning_angle);
      int sign = (turning_angle >= 0) ? 1 : -1;
      double delta_x = radius * sin(abs(turning_angle));
      double delta_y = sign * radius * (1 - cos(turning_angle));
      trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
      trajopt->AddBoundingBoxConstraint(delta_x, delta_x,
                                        xf(pos_map.at("base_x")));
      trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_y")));
      trajopt->AddBoundingBoxConstraint(delta_y, delta_y,
                                        xf(pos_map.at("base_y")));

      // floating base velocity constraint at mid point
      // TODO: double check that the floating base velocity is wrt local frame
      /*trajopt->AddBoundingBoxConstraint(stride_length / duration,
                                        stride_length / duration,
                                        x_mid(n_q + vel_map.at("base_vx")));
      trajopt->AddBoundingBoxConstraint(0, 0, x_mid(n_q +
      vel_map.at("base_vy"))); trajopt->AddBoundingBoxConstraint(turning_rate,
                                        turning_rate,
                                        x_mid(n_q + vel_map.at("base_wz")));*/

      // velocity constraint at the end point
      // TODO: double check that the floating base velocity is wrt local frame
      if (periodic_floating_base_vel) {
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
                                     xf(n_q + vel_map.at("base_wx")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
                                     -xf(n_q + vel_map.at("base_wy")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
                                     xf(n_q + vel_map.at("base_wz")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
                                     xf(n_q + vel_map.at("base_vx")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                                     -xf(n_q + vel_map.at("base_vy")));
        trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
                                     xf(n_q + vel_map.at("base_vz")));
      }
    }
  }

  // The legs joint positions/velocities/torque should be mirrored between legs
  // (notice that hip yaw and roll should be asymmetric instead of symmetric.)
  for (const auto& l_r_pair : l_r_pairs) {
    // Asymmetrical joints
    for (const auto& asy_joint_name : asy_joint_names) {
      // positions
      if (periodic_joint_pos) {
        trajopt->AddLinearConstraint(
            x0(pos_map.at(asy_joint_name + l_r_pair.first)) >=
            -xf(pos_map.at(asy_joint_name + l_r_pair.second)) - eps_pos_period);
        trajopt->AddLinearConstraint(
            x0(pos_map.at(asy_joint_name + l_r_pair.first)) <=
            -xf(pos_map.at(asy_joint_name + l_r_pair.second)) + eps_pos_period);
      }
      // velocities
      if (periodic_joint_vel) {
        trajopt->AddLinearConstraint(
            x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) >=
            -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")) -
                eps_vel_period);
        trajopt->AddLinearConstraint(
            x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) <=
            -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")) +
                eps_vel_period);
      }
      // inputs
      if (periodic_effort) {
        trajopt->AddLinearConstraint(
            u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
            -uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
      }
    }
    // Symmetrical joints
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      // positions
      if (periodic_joint_pos) {
        trajopt->AddLinearConstraint(
            x0(pos_map.at(sym_joint_names[i] + l_r_pair.first)) >=
            xf(pos_map.at(sym_joint_names[i] + l_r_pair.second)) -
                eps_pos_period);
        trajopt->AddLinearConstraint(
            x0(pos_map.at(sym_joint_names[i] + l_r_pair.first)) <=
            xf(pos_map.at(sym_joint_names[i] + l_r_pair.second)) +
                eps_pos_period);
      }
      // velocities
      if (periodic_joint_vel) {
        trajopt->AddLinearConstraint(
            x0(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot")) >=
            xf(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")) -
                eps_vel_period);
        trajopt->AddLinearConstraint(
            x0(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot")) <=
            xf(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")) +
                eps_vel_period);
      }
      // inputs (ankle joint is not actuated)
      if (periodic_effort) {
        if (sym_joint_names[i] != "ankle_joint") {
          trajopt->AddLinearConstraint(
              u0(act_map.at(sym_joint_names[i] + l_r_pair.first + "_motor")) ==
              uf(act_map.at(sym_joint_names[i] + l_r_pair.second + "_motor")));
        }
      }
    }
  }  // end for (l_r_pairs)

  // joint limits
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // u limit
  if (pre_and_post_impact_efforts) {
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        auto ui = trajopt->input_vars_by_mode(mode, index);
        trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                          VectorXd::Constant(n_u, +300), ui);
      }
    }
  } else {
    for (int i = 0; i < N; i++) {
      auto ui = trajopt->input(i);
      trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                        VectorXd::Constant(n_u, +300), ui);
    }
  }

  if (setting.com_accel_constraint && zero_com_height_vel) {
    cout << "Adding zero COM height acceleration constraint\n";
    auto com_vel_constraint = std::make_shared<ComHeightVelConstraint>(&plant);
    std::unordered_map<int, double> com_vel_constraint_scale;
    com_vel_constraint_scale.insert(std::pair<int, double>(0, 0.1));
    com_vel_constraint->SetConstraintScaling(com_vel_constraint_scale);
    for (int index = 0; index < num_time_samples[0] - 1; index++) {
      auto x0 = trajopt->state(index);
      auto x1 = trajopt->state(index + 1);
      trajopt->AddConstraint(com_vel_constraint, {x0, x1});
    }
  } else {
    zero_com_height_vel = false;
  }

  // Testing
  if (zero_com_height_vel_difference) {
    cout << "Adding zero COM height vel constraint\n";
    auto com_zero_vel_constraint =
        std::make_shared<ComZeroHeightVelConstraint>(plant);
    //  std::unordered_map<int, double> com_zero_vel_constraint_scale;
    //  com_zero_vel_constraint_scale.insert(std::pair<int, double>(0, 0.1));
    //  com_zero_vel_constraint->SetConstraintScaling(com_zero_vel_constraint_scale);
    //    std::vector<int> index_list = {0,
    //                                   1,
    //                                   2,
    //                                   num_time_samples[0] - 3,
    //                                   num_time_samples[0] - 2,
    //                                   num_time_samples[0] - 1};
    //    for (auto index : index_list) {
    for (int index = 0; index < num_time_samples[0]; index++) {
      auto xi = trajopt->state(index);
      trajopt->AddConstraint(com_zero_vel_constraint, xi);
    }
  }
  if (zero_pelvis_height_vel) {
    cout << "Adding zero pelvis height vel constraint\n";
    for (int index = 0; index < num_time_samples[0]; index++) {
      auto xi = trajopt->state(index);
      trajopt->AddBoundingBoxConstraint(0, 0, xi(n_q + vel_map.at("base_vz")));
    }
  }

  // toe position constraint in y direction (avoid leg crossing)
  VectorXd one = VectorXd::Ones(1);
  std::unordered_map<int, double> odbp_constraint_scale;  // scaling
  odbp_constraint_scale.insert(std::pair<int, double>(0, s));
  if (swing_leg_collision_avoidance && (turning_rate == 0)) {
    auto left_foot_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_left", Vector3d::Zero(),
            MatrixXd::Identity(3, 3).row(1), 0.05 * one,
            std::numeric_limits<double>::infinity() * one,
            "left_foot_constraint_y");
    auto right_foot_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_right", Vector3d::Zero(),
            MatrixXd::Identity(3, 3).row(1),
            -std::numeric_limits<double>::infinity() * one, -0.05 * one,
            "right_foot_constraint_y");
    // scaling
    left_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
    right_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
    for (int index = 0; index < num_time_samples[0]; index++) {
      auto x = trajopt->state(index);
      trajopt->AddConstraint(left_foot_constraint, x.head(n_q));
      trajopt->AddConstraint(right_foot_constraint, x.head(n_q));
    }
  } else {
    swing_leg_collision_avoidance = false;
  }

  // testing - toe height constraint at mid stance (avoid foot scuffing)
  Vector3d z_hat(0, 0, 1);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(z_hat, ground_normal);
  Eigen::Matrix3d T_ground_incline = q.matrix().transpose();
  /*  auto right_foot_constraint_z0 =
    std::make_shared<PointPositionConstraint<double>>( plant, "toe_right",
    Vector3d::Zero(), T_ground_incline.row(2), 0.1 * one,
        std::numeric_limits<double>::infinity() * one);
    auto x_mid = trajopt->state(num_time_samples[0] / 2);
    trajopt->AddConstraint(right_foot_constraint_z0, x_mid.head(n_q));*/

  // testing -- swing foot contact point height constraint
  if (swing_foot_ground_clearance) {
    for (int index = 0; index < num_time_samples[0] - 1; index++) {
      double h_min = 0;
      //    if (index == int(num_time_samples[0] / 2)) {
      //      h_min = 0.1;  // 10 centimeter high in the mid point
      //    }

      double h_max = std::numeric_limits<double>::infinity();
      if (index == 0) {
        h_max = 0;
      }

      auto x_i = trajopt->state(index);

      auto right_foot_constraint_z1 =
          std::make_shared<PointPositionConstraint<double>>(
              plant, "toe_right", pt_front_contact, T_ground_incline.row(2),
              h_min * one, h_max * one, "right_foot_constraint_z");
      auto right_foot_constraint_z2 =
          std::make_shared<PointPositionConstraint<double>>(
              plant, "toe_right", pt_rear_contact, T_ground_incline.row(2),
              h_min * one, h_max * one);

      // scaling
      right_foot_constraint_z1->SetConstraintScaling(odbp_constraint_scale);
      right_foot_constraint_z2->SetConstraintScaling(odbp_constraint_scale);

      trajopt->AddConstraint(right_foot_constraint_z1, x_i.head(n_q));
      trajopt->AddConstraint(right_foot_constraint_z2, x_i.head(n_q));
    }
  }

  //  // testing -- vertical touchdown velocity
  //  trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(0) == 0);
  //  trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(3) == 0);
  //  // testing -- prevent backward foot velocity
  //  auto right_foot_vel_constraint =
  //  std::make_shared<PointVelocityConstraint<double>>(
  //      plant, "toe_right", Vector3d::Zero(), T_ground_incline.row(0),
  //      0 , std::numeric_limits<double>::infinity());
  //  for (int index = 1; index < num_time_samples[0] - 1; index++) {
  //    auto x_i = trajopt->state(index);
  //    trajopt->AddConstraint(right_foot_vel_constraint, x_i);
  //  }

  // testing -- zero impact
  if (setting.is_zero_touchdown_impact) {
    trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(2) == 0);
    trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(5) == 0);
  }

  // testing -- swing foot pos at mid stance is the average of the start and the
  // end of the stance
  //  if (turning_rate == 0) {
  //  auto swing_foot_mid_stance_xy_constraint =
  //      std::make_shared<SwingFootXYPosAtMidStanceConstraint>(&plant,
  //      "toe_right",
  //                                                            Vector3d::Zero());
  //  trajopt->AddConstraint(swing_foot_mid_stance_xy_constraint,
  //                         {x0.head(n_q), x_mid.head(n_q), xf.head(n_q)});
  //  }

  // testing -- lock the swing toe joint position (otherwise it shakes too much)
  // Somehow the swing toe doesn't shake that much anymore after adding
  // constraint such that the two contact points has to be above the ground?
  /*auto d = trajopt->NewContinuousVariables(1, "d_toe");
  for (int index = 0; index < num_time_samples[0] - 1; index++) {
    auto x0 = trajopt->state(index);
    auto x1 = trajopt->state(index + 1);
    trajopt->AddLinearConstraint(x0(n_q - 1) + d(0) == x1(n_q - 1));
  }*/

  // Testing -- constraint on normal force
  if (ground_normal_force_margin) {
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        auto lambda = trajopt->force(mode, index);
        trajopt->AddLinearConstraint(lambda(2) >= 10);
        trajopt->AddLinearConstraint(lambda(5) >= 10);
      }
    }
  }
  // Testing -- constraint left four-bar force (seems to help in high speed)
  if (constrain_stance_leg_fourbar_force) {
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        if (four_bar_in_right_support) {
          auto lambda = trajopt->force(mode, index);
          trajopt->AddLinearConstraint(lambda(6) <= 0);  // left leg four bar
        }
      }
    }
  }

  // Testing -- add lower bound for the force at the first/last knot points
  if (lower_bound_on_ground_reaction_force_at_the_first_and_last_knot) {
    int mode = 0;
    vector<int> index_list = {0, num_time_samples[mode] - 1};
    for (int index : index_list) {
      auto lambda = trajopt->force(mode, index);
      trajopt->AddLinearConstraint(lambda(2) + lambda(5) >= 220);
    }
  }

  // Testing -- add constraint on joint acceleration
  // TODO: include second mode?
  if (add_joint_acceleration_constraint) {
    auto joint_accel_constraint = std::make_shared<JointAccelConstraint>(
        joint_accel_lb * VectorXd::Ones(n_v),
        joint_accel_ub * VectorXd::Ones(n_v), plant, dataset_list[0],
        "joint_accel_constraint");
    // scaling
    // TODO: set constraint scaling
    // joint_accel_constraint->SetConstraintScaling(odbp_constraint_scale);
    for (int i = 0; i < N; i++) {
      auto x0 = trajopt->state(i);
      auto u0 = trajopt->input(i);
      auto l0 = trajopt->force(0, i);
      trajopt->AddConstraint(joint_accel_constraint, {x0, u0, l0});
    }
  }

  // Testing -- add constraint to limit hip roll position
  if (add_hip_roll_pos_constraint) {
    for (const auto& l_r_pair : l_r_pairs) {
      for (int index = 0; index < num_time_samples[0]; index++) {
        auto xi = trajopt->state(index);
        trajopt->AddBoundingBoxConstraint(
            hip_roll_pos_lb, hip_roll_pos_ub,
            xi(pos_map.at("hip_roll" + l_r_pair.first)));
      }
    }
  }

  // Testing -- add constraint to limit base y velocity
  if (add_base_vy_constraint) {
    for (int index = 0; index < num_time_samples[0]; index++) {
      auto xi = trajopt->state(index);
      trajopt->AddBoundingBoxConstraint(base_vy_lb, base_vy_ub,
                                        xi(n_q + vel_map.at("base_vy")));
    }
  }

  // Scale decision variable
  std::vector<int> idx_list;
  // time
  trajopt->ScaleTimeVariables(0.008);
  // state
  trajopt->ScaleStateVariables({0, 1, 2, 3}, 0.5);
  idx_list.clear();
  for (int i = n_q; i < n_q + n_v - 2; i++) {
    idx_list.push_back(i);
  }
  trajopt->ScaleStateVariables(idx_list, 10);
  trajopt->ScaleStateVariables({n_q + n_v - 2, n_q + n_v - 2}, 10);
  trajopt->ScaleStateVariables({n_q + n_v - 1, n_q + n_v - 1}, 10);
  // input
  trajopt->ScaleInputVariables({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, 100);
  if (pre_and_post_impact_efforts) {
    auto u_post_impact = trajopt->u_post_impact_vars_by_mode(0);
    for (int idx = 0; idx < u_post_impact.size(); idx++) {
      trajopt->SetVariableScaling(u_post_impact(idx), 100);
    }
  }
  // force
  idx_list.clear();
  for (int i = 0; i < ls_dataset.countConstraintsWithoutSkipping(); i++) {
    idx_list.push_back(i);
  }
  trajopt->ScaleForceVariables(0, idx_list, 1000);
  idx_list.clear();
  for (int i = 0; i < rs_dataset.countConstraintsWithoutSkipping(); i++) {
    idx_list.push_back(i);
  }
  if (!only_one_mode) trajopt->ScaleForceVariables(1, idx_list, 1000);
  // impulse
  if (!only_one_mode) trajopt->ScaleImpulseVariables(0, idx_list, 10);  // 0.1
  // quaternion slack
  trajopt->ScaleQuaternionSlackVariables(30);
  // Constraint slack
  trajopt->ScaleKinConstraintSlackVariables(0, {0, 1, 2, 3, 4, 5}, 50);
  trajopt->ScaleKinConstraintSlackVariables(0, {6, 7}, 500);

  // add cost
  /*trajopt->AddRunningCost(x.tail(n_v).transpose() * W_Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * W_R * u);*/
  // Add cost without time
  auto fixed_dt = duration / (N - 1);
  for (int i = 0; i < N - 1; i++) {
    auto v0 = trajopt->state(i).tail(n_v);
    auto v1 = trajopt->state(i + 1).tail(n_v);
    trajopt->AddCost(((v0.transpose() * W_Q * v0) * fixed_dt / 2)(0));
    trajopt->AddCost(((v1.transpose() * W_Q * v1) * fixed_dt / 2)(0));
  }
  for (int i = 0; i < N - 1; i++) {
    auto u0 = trajopt->input(i);
    auto u1 = trajopt->input(i + 1);
    trajopt->AddCost(((u0.transpose() * W_R * u0) * fixed_dt / 2)(0));
    trajopt->AddCost(((u1.transpose() * W_R * u1) * fixed_dt / 2)(0));
  }
  if (pre_and_post_impact_efforts) {
    // This block assume that there is only once knot in the second mode
    auto u_post_impact = trajopt->u_post_impact_vars_by_mode(0);
    trajopt->AddCost(
        ((u_post_impact.transpose() * W_R * u_post_impact) * fixed_dt / 2)(0));
  }
  // Not use trapozoidal integration
  if (not_trapo_integration_cost) {
    auto v0 = trajopt->state(0).tail(n_v);
    auto v1 = trajopt->state(N - 1).tail(n_v);
    trajopt->AddCost(((v0.transpose() * W_Q * v0) * fixed_dt / 2)(0));
    trajopt->AddCost(((v1.transpose() * W_Q * v1) * fixed_dt / 2)(0));
    auto u0 = trajopt->input(0);
    auto u1 = trajopt->input(N - 1);
    trajopt->AddCost(((u0.transpose() * W_R * u0) * fixed_dt / 2)(0));
    trajopt->AddCost(((u1.transpose() * W_R * u1) * fixed_dt / 2)(0));
  }

  // Testing -- Make the last knot point weight much bigger
  bool much_bigger_weight_at_last_knot = false;
  if (much_bigger_weight_at_last_knot) {
    double multiplier = 200;
    auto v1 = trajopt->state(N - 1).tail(n_v);
    trajopt->AddCost(
        ((v1.transpose() * W_Q * v1) * multiplier * fixed_dt / 2)(0));
    auto u1 = trajopt->input(N - 1);
    trajopt->AddCost(
        ((u1.transpose() * W_R * u1) * multiplier * fixed_dt / 2)(0));
  }

  // Testing
  //  auto v0 = trajopt->state(0).tail(n_v);
  //  auto v1 = trajopt->state(N - 1).tail(n_v);
  //  trajopt->AddCost(((v0.transpose() * W_Q * v0) * 100 * fixed_dt / 2)(0));
  //  trajopt->AddCost(((v1.transpose() * W_Q * v1) * 100 * fixed_dt / 2)(0));

  // add cost on force difference wrt time
  bool diff_with_force_at_collocation = false;
  if (w_lambda_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = trajopt->force(0, i);
      auto lambda1 = trajopt->force(0, i + 1);
      auto lambdac = trajopt->collocation_force(0, i);
      if (diff_with_force_at_collocation) {
        trajopt->AddCost(w_lambda_diff *
                         (lambda0 - lambdac).dot(lambda0 - lambdac));
        trajopt->AddCost(w_lambda_diff *
                         (lambdac - lambda1).dot(lambdac - lambda1));
      } else {
        trajopt->AddCost(w_lambda_diff *
                         (lambda0 - lambda1).dot(lambda0 - lambda1));
      }
    }
  }
  // add cost on vel difference wrt time
  MatrixXd Q_q_diff = w_q_diff * MatrixXd::Identity(n_q, n_q);
  Q_q_diff(n_q - 1, n_q - 1) = w_q_diff_swing_toe;
  if (w_q_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto q0 = trajopt->state(i).head(n_q);
      auto q1 = trajopt->state(i + 1).head(n_q);
      trajopt->AddCost((q0 - q1).dot(Q_q_diff * (q0 - q1)));
    }
  }
  // add cost on pos difference wrt time
  MatrixXd Q_v_diff = w_v_diff * MatrixXd::Identity(n_v, n_v);
  Q_v_diff(7, 7) = w_v_diff_swing_leg;
  Q_v_diff(9, 9) = w_v_diff_swing_leg;
  Q_v_diff(11, 11) = w_v_diff_swing_leg;
  Q_v_diff(13, 13) = w_v_diff_swing_leg;
  Q_v_diff(15, 15) = w_v_diff_swing_leg;
  Q_v_diff(17, 17) = w_v_diff_swing_leg;
  if (w_v_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto v0 = trajopt->state(i).tail(n_v);
      auto v1 = trajopt->state(i + 1).tail(n_v);
      trajopt->AddCost((v0 - v1).dot(Q_v_diff * (v0 - v1)));
    }
  }
  // add cost on input difference wrt time
  if (w_u_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto u0 = trajopt->input(i);
      auto u1 = trajopt->input(i + 1);
      trajopt->AddCost(w_u_diff * (u0 - u1).dot(u0 - u1));
    }
  }
  // add cost on joint position
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(w_q_hip_roll * q.transpose() * q);
    }
  }
  if (w_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(9, 2);
      trajopt->AddCost(w_q_hip_yaw * q.transpose() * q);
    }
  }
  if (w_q_quat) {
    for (int i = 0; i < N; i++) {
      // get desired turning rate at knot point i
      double turning_rate_i = turning_rate * i / (N - 1);
      VectorXd desired_quat(4);
      desired_quat << cos(turning_rate_i / 2), 0, 0, sin(turning_rate_i / 2);

      auto quat = trajopt->state(i).head(4);
      trajopt->AddCost(w_q_quat * (quat - desired_quat).transpose() *
                       (quat - desired_quat));
    }
  }

  // Add cost on  collocation force, to eliminate the forces
  // in the null space of constraints
  for (unsigned int i = 0; i < num_time_samples.size(); i++) {
    if (options_list[i].getForceCost() != 0) {
      int n_lambda = dataset_list[i]->countConstraintsWithoutSkipping();
      auto A = options_list[i].getForceCost() *
               MatrixXd::Identity(n_lambda, n_lambda);
      auto b = MatrixXd::Zero(n_lambda, 1);
      for (int j = 0; j < num_time_samples[i] - 1; j++) {
        // Add || Ax - b ||^2
        trajopt->AddL2NormCost(A, b, trajopt->collocation_force(i, j));
      }
    }
  }

  // Testing: add velocity at collocation point to the cost function
  if (add_cost_on_collocation_vel) {
    auto vel_collocation_cost = std::make_shared<CollocationVelocityCost>(
        0.1 * W_Q, plant, dataset_list[0]);
    for (int i = 0; i < N - 1; i++) {
      auto dt = trajopt->timestep(i);
      auto x0 = trajopt->state(i);
      auto x1 = trajopt->state(i + 1);
      auto u0 = trajopt->input(i);
      auto u1 = trajopt->input(i + 1);
      auto l0 = trajopt->force(0, i);
      auto l1 = trajopt->force(0, i + 1);

      trajopt->AddCost(vel_collocation_cost, {dt, x0, x1, u0, u1, l0, l1});
    }
  }

  // Testing: add joint acceleration cost at knot points
  auto joint_accel_cost = std::make_shared<JointAccelCost>(
      w_joint_accel * W_Q, plant, dataset_list[0]);
  if (add_joint_acceleration_cost) {
    int idx_start = 0;
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        auto xi = trajopt->state_vars_by_mode(mode, index);
        auto ui = (only_one_mode || !pre_and_post_impact_efforts)
                      ? trajopt->input(idx_start + index)
                      : trajopt->input_vars_by_mode(mode, index);
        auto li = trajopt->force(mode, index);
        trajopt->AddCost(joint_accel_cost, {xi, ui, li});
      }
      idx_start += num_time_samples[mode] - 1;
    }
  }

  // Move the trajectory optimization problem into GoldilocksModelTrajOpt
  // where we add the constraints for reduced order model
  GoldilocksModelTrajOpt gm_traj_opt(
      rom, std::move(trajopt), plant, num_time_samples, dataset_list,
      is_get_nominal, setting, rom_option, robot_option, s);

  addRegularization(is_get_nominal, setting.eps_reg, gm_traj_opt);

  // initial guess if the file exists
  if (!setting.init_file.empty()) {
    setInitialGuessFromFile(setting.directory, setting.init_file, gm_traj_opt);
  } else {
    // Do inverse kinematics to get q initial guess
    vector<VectorXd> q_seed =
        GetCassieInitGuessForQ(N, stride_length, ground_incline, plant);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed =
        GetCassieInitGuessForV(q_seed, duration / (N - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = gm_traj_opt.dircon->state(i);
      VectorXd xi_seed(n_q + n_v);
      xi_seed << q_seed.at(i), v_seed.at(i);
      gm_traj_opt.dircon->SetInitialGuess(xi, xi_seed);
    }
  }
  // Careful: MUST set the initial guess for quaternion, since 0-norm quaternion
  // produces NAN value in some calculation.
  for (int i = 0; i < N; i++) {
    auto xi = gm_traj_opt.dircon->state(i);
    if ((gm_traj_opt.dircon->GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(gm_traj_opt.dircon->GetInitialGuess(xi.head(4)).norm())) {
      gm_traj_opt.dircon->SetInitialGuess(xi(0), 1);
      gm_traj_opt.dircon->SetInitialGuess(xi(1), 0);
      gm_traj_opt.dircon->SetInitialGuess(xi(2), 0);
      gm_traj_opt.dircon->SetInitialGuess(xi(3), 0);
    }
  }

  // Print variable scaling
  /*for (int i = 0; i < gm_traj_opt.dircon->decision_variables().size(); i++) {
    cout << gm_traj_opt.dircon->decision_variable(i) << ", ";
    cout << gm_traj_opt.dircon->decision_variable(i).get_id() << ", ";
    cout <<
  gm_traj_opt.dircon->FindDecisionVariableIndex(gm_traj_opt.dircon->decision_variable(i))
         << ", ";
    auto scale_map = gm_traj_opt.dircon->GetVariableScaling();
    auto it = scale_map.find(i);
    if (it != scale_map.end()) {
      cout << it->second;
    } else {
      cout << "none";
    }
    cout << ", ";
    cout <<
  gm_traj_opt.dircon->GetInitialGuess(gm_traj_opt.dircon->decision_variable(i));
    cout << endl;
  }*/

  // Testing
  //   cout << "Choose the best solver: " <<
  //        drake::solvers::ChooseBestSolver(*(gm_traj_opt.dircon)).name() <<
  //        endl;

  drake::solvers::SolverId solver_id("");

  if (setting.use_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    //    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(*gm_traj_opt.dircon);
    //    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  // Testing -- visualize poses
  if (sample_idx == 0) {
    double alpha = 1;
    gm_traj_opt.dircon->CreateVisualizationCallback(
        "examples/Cassie/urdf/cassie_fixed_springs.urdf", 5, alpha);
  }

  // cout << "Solving DIRCON (based on MultipleShooting)\n";
  auto start = std::chrono::high_resolution_clock::now();
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(*gm_traj_opt.dircon, gm_traj_opt.dircon->initial_guess(),
                gm_traj_opt.dircon->solver_options(), &result);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  // Save trajectory to file
  if (sample_idx == 0) {
    string file_name = "dircon_trajectory";
    DirconTrajectory saved_traj(
        plant, *gm_traj_opt.dircon, result, file_name,
        "Decision variables and state/input trajectories");
    saved_traj.WriteToFile(setting.directory + file_name);
    std::cout << "Wrote to file: " << setting.directory + file_name
              << std::endl;
  }

  bool is_print_for_debugging = false;
  VectorXd w_sol;
  extractResult(w_sol, gm_traj_opt, result, elapsed, num_time_samples, N, plant,
                plant_autoDiff, setting, rom, task, QPs, sample_idx, n_rerun,
                cost_threshold_for_update, N_rerun, dataset_list,
                is_print_for_debugging);
  postProcessing(w_sol, gm_traj_opt, result, num_time_samples, N, plant,
                 plant_autoDiff, setting, rom, QPs, is_get_nominal,
                 extend_model, sample_idx, n_rerun, cost_threshold_for_update,
                 N_rerun, rom_option, robot_option);

  if (sample_idx == 0) {
    is_print_for_debugging = true;
  }

  if (is_print_for_debugging) {
    // Impulse variable's value
    for (int i = w_sol.size() - rs_dataset.countConstraints() - 1;
         i < w_sol.size(); i++) {
      cout << i << ": " << gm_traj_opt.dircon->decision_variables()[i] << ", "
           << w_sol[i] << endl;
    }
    cout << endl;

    // Extract result for printing
    VectorXd time_at_knots = gm_traj_opt.dircon->GetSampleTimes(result);
    MatrixXd state_at_knots = gm_traj_opt.dircon->GetStateSamples(result);
    MatrixXd input_at_knots = gm_traj_opt.dircon->GetInputSamples(result);

    // Print weight
    cout << "\nw_Q = " << w_Q << endl;
    cout << "w_Q_vy = " << w_Q_vy << endl;
    cout << "w_Q_vz = " << w_Q_vz << endl;
    cout << "w_Q_swing_toe = " << w_Q_swing_toe << endl;
    cout << "w_R = " << w_R << endl;
    cout << "w_R_swing_toe = " << w_R_swing_toe << endl;
    cout << "w_lambda = " << w_lambda << endl;
    cout << "w_lambda_diff = " << w_lambda_diff << endl;
    cout << "w_q_diff = " << w_q_diff << endl;
    cout << "w_q_diff_swing_toe = " << w_q_diff_swing_toe << endl;
    cout << "w_v_diff = " << w_v_diff << endl;
    cout << "w_v_diff_swing_leg = " << w_v_diff_swing_leg << endl;
    cout << "w_u_diff = " << w_u_diff << endl;
    cout << "w_q_hip_roll = " << w_q_hip_roll << endl;
    cout << "w_q_hip_yaw = " << w_q_hip_yaw << endl;
    cout << "w_q_quat = " << w_q_quat << endl;
    cout << "w_joint_accel = " << w_joint_accel << endl;

    // Calculate each term of the cost
    double total_cost = 0;
    double cost_x = 0;
    for (int i = 0; i < N - 1; i++) {
      auto v0 = state_at_knots.col(i).tail(n_v);
      auto v1 = state_at_knots.col(i + 1).tail(n_v);
      auto h = time_at_knots(i + 1) - time_at_knots(i);
      cost_x += ((v0.transpose() * W_Q * v0) * h / 2)(0);
      cost_x += ((v1.transpose() * W_Q * v1) * h / 2)(0);
    }
    total_cost += cost_x;
    cout << "cost_x = " << cost_x << endl;
    double cost_u = 0;
    for (int i = 0; i < N - 1; i++) {
      auto u0 = input_at_knots.col(i);
      auto u1 = input_at_knots.col(i + 1);
      auto h = time_at_knots(i + 1) - time_at_knots(i);
      cost_u += ((u0.transpose() * W_R * u0) * h / 2)(0);
      cost_u += ((u1.transpose() * W_R * u1) * h / 2)(0);
    }
    if (pre_and_post_impact_efforts) {
      auto u_post_impact =
          result.GetSolution(gm_traj_opt.dircon->u_post_impact_vars_by_mode(0));
      cost_u +=
          ((u_post_impact.transpose() * W_R * u_post_impact) * fixed_dt / 2)(0);
    }
    total_cost += cost_u;
    cout << "cost_u = " << cost_u << endl;
    double cost_lambda = 0;
    for (unsigned int i = 0; i < num_time_samples.size(); i++) {
      for (int j = 0; j < num_time_samples[i]; j++) {
        auto lambda = result.GetSolution(gm_traj_opt.dircon->force(i, j));
        cost_lambda += (options_list[i].getForceCost() * lambda).squaredNorm();
      }
    }
    total_cost += cost_lambda;
    cout << "cost_lambda (at knots) = " << cost_lambda << endl;
    // TODO: Sum collocation force cost
    cout << "cost_lambda (at collocation points) = ..." << endl;
    // cost on force difference wrt time
    double cost_lambda_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = result.GetSolution(gm_traj_opt.dircon->force(0, i));
      auto lambda1 = result.GetSolution(gm_traj_opt.dircon->force(0, i + 1));
      auto lambdac =
          result.GetSolution(gm_traj_opt.dircon->collocation_force(0, i));
      if (diff_with_force_at_collocation) {
        cost_lambda_diff +=
            w_lambda_diff * (lambda0 - lambdac).dot(lambda0 - lambdac);
        cost_lambda_diff +=
            w_lambda_diff * (lambdac - lambda1).dot(lambdac - lambda1);
      } else {
        cost_lambda_diff +=
            w_lambda_diff * (lambda0 - lambda1).dot(lambda0 - lambda1);
      }
    }
    total_cost += cost_lambda_diff;
    cout << "cost_lambda_diff = " << cost_lambda_diff << endl;
    // cost on pos difference wrt time
    double cost_pos_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto q0 = result.GetSolution(gm_traj_opt.dircon->state(i).head(n_q));
      auto q1 = result.GetSolution(gm_traj_opt.dircon->state(i + 1).head(n_q));
      cost_pos_diff += (q0 - q1).dot(Q_q_diff * (q0 - q1));
    }
    total_cost += cost_pos_diff;
    cout << "cost_pos_diff = " << cost_pos_diff << endl;
    // cost on vel difference wrt time
    double cost_vel_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto v0 = result.GetSolution(gm_traj_opt.dircon->state(i).tail(n_v));
      auto v1 = result.GetSolution(gm_traj_opt.dircon->state(i + 1).tail(n_v));
      cost_vel_diff += (v0 - v1).dot(Q_v_diff * (v0 - v1));
    }
    total_cost += cost_vel_diff;
    cout << "cost_vel_diff = " << cost_vel_diff << endl;
    // cost on input difference wrt time
    double cost_u_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto u0 = result.GetSolution(gm_traj_opt.dircon->input(i));
      auto u1 = result.GetSolution(gm_traj_opt.dircon->input(i + 1));
      cost_u_diff += w_u_diff * (u0 - u1).dot(u0 - u1);
    }
    total_cost += cost_u_diff;
    cout << "cost_u_diff = " << cost_u_diff << endl;
    // add cost on joint position
    double cost_q_hip_roll = 0;
    for (int i = 0; i < N; i++) {
      auto q = result.GetSolution(gm_traj_opt.dircon->state(i).segment(7, 2));
      cost_q_hip_roll += w_q_hip_roll * q.transpose() * q;
    }
    total_cost += cost_q_hip_roll;
    cout << "cost_q_hip_roll = " << cost_q_hip_roll << endl;
    double cost_q_hip_yaw = 0;
    for (int i = 0; i < N; i++) {
      auto q = result.GetSolution(gm_traj_opt.dircon->state(i).segment(9, 2));
      cost_q_hip_yaw += w_q_hip_yaw * q.transpose() * q;
    }
    total_cost += cost_q_hip_yaw;
    cout << "cost_q_hip_yaw = " << cost_q_hip_yaw << endl;
    // add cost on quaternion
    double cost_q_quat_xyz = 0;
    for (int i = 0; i < N; i++) {
      // get desired turning rate at knot point i
      double turning_rate_i = turning_rate * i / (N - 1);
      VectorXd desired_quat(4);
      desired_quat << cos(turning_rate_i / 2), 0, 0, sin(turning_rate_i / 2);

      auto quat = result.GetSolution(gm_traj_opt.dircon->state(i).head(4));
      cost_q_quat_xyz +=
          w_q_quat * (quat - desired_quat).transpose() * (quat - desired_quat);
    }
    total_cost += cost_q_quat_xyz;
    cout << "cost_q_quat_xyz = " << cost_q_quat_xyz << endl;
    double cost_joint_acceleration = 0;
    if (add_joint_acceleration_cost) {
      int idx_start = 0;
      for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
        for (int index = 0; index < num_time_samples[mode]; index++) {
          auto xi = result.GetSolution(
              gm_traj_opt.dircon->state_vars_by_mode(mode, index));
          auto ui = result.GetSolution(
              (only_one_mode || !pre_and_post_impact_efforts)
                  ? gm_traj_opt.dircon->input(idx_start + index)
                  : gm_traj_opt.dircon->input_vars_by_mode(mode, index));
          auto li = result.GetSolution(gm_traj_opt.dircon->force(mode, index));

          Eigen::VectorXd x_val(xi.size() + ui.size() + li.size());
          x_val << xi, ui, li;
          Eigen::VectorXd y_val(1);
          joint_accel_cost->Eval(x_val, &y_val);
          cost_joint_acceleration += y_val(0);
        }
        idx_start += num_time_samples[mode] - 1;
      }
    }
    total_cost += cost_joint_acceleration;
    cout << "cost_joint_acceleration = " << cost_joint_acceleration << endl;

    cout << "total_cost (only the nominal traj cost terms) = " << total_cost
         << endl;

    // Constraints
    cout << endl;
    cout << "swing_foot_ground_clearance = " << swing_foot_ground_clearance
         << endl;
    cout << "swing_leg_collision_avoidance = " << swing_leg_collision_avoidance
         << endl;
    cout << "periodic_quaternion (only effective when turning rate = 0) = "
         << periodic_quaternion << endl;
    cout << "periodic_joint_pos = " << periodic_joint_pos << endl;
    cout << "periodic_floating_base_vel = " << periodic_floating_base_vel
         << endl;
    cout << "periodic_joint_vel = " << periodic_joint_vel << endl;
    cout << "periodic_effort = " << periodic_effort << endl;
    cout << "ground_normal_force_margin = " << ground_normal_force_margin
         << endl;
    cout << "zero_com_height_vel = " << zero_com_height_vel << endl;
    cout << "zero_com_height_vel_difference = "
         << zero_com_height_vel_difference << endl;
    cout << "zero_pelvis_height_vel = " << zero_pelvis_height_vel << endl;
    cout << "constrain_stance_leg_fourbar_force = "
         << constrain_stance_leg_fourbar_force << endl;
    cout << "four_bar_in_right_support = " << four_bar_in_right_support << endl;

    cout << endl;
    cout << "only_one_mode = " << only_one_mode << endl;
    if (only_one_mode) {
      if (one_mode_full_states_constraint_at_boundary) {
        cout << "  one_mode_full_states_constraint_at_boundary" << endl;
      } else if (one_mode_full_positions_constraint_at_boundary) {
        cout << "  one_mode_full_positions_constraint_at_boundary" << endl;
      } else if (one_mode_base_x_constraint_at_boundary) {
        cout << "  one_mode_base_x_constraint_at_boundary" << endl;
      }
    }
    cout << "add_cost_on_collocation_vel = " << add_cost_on_collocation_vel
         << endl;
    cout << "add_joint_acceleration_cost = " << add_joint_acceleration_cost
         << endl;
    cout << "lower_bound_on_ground_reaction_force_at_the_first_and_last_knot = "
         << lower_bound_on_ground_reaction_force_at_the_first_and_last_knot
         << endl;
    cout << "not_trapo_integration_cost = " << not_trapo_integration_cost
         << endl;
    cout << "add_joint_acceleration_constraint = "
         << add_joint_acceleration_constraint
         << "; (lb, ub) = " << joint_accel_lb << ", " << joint_accel_ub << endl;
    cout << "add_hip_roll_pos_constraint = " << add_hip_roll_pos_constraint
         << "; (lb, ub) = " << hip_roll_pos_lb << ", " << hip_roll_pos_ub
         << endl;
    cout << "add_base_vy_constraint = " << add_base_vy_constraint
         << "; (lb, ub) = " << base_vy_lb << ", " << base_vy_ub << endl;

    cout << "pre_and_post_impact_efforts = " << pre_and_post_impact_efforts
         << endl;
    if (pre_and_post_impact_efforts) {
      cout << "u_pre_impact = "
           << result.GetSolution(gm_traj_opt.dircon->input(N - 1)).transpose()
           << endl;
      cout << "u_post_impact = "
           << result
                  .GetSolution(gm_traj_opt.dircon->input_vars_by_mode(
                      num_time_samples.size() - 1,
                      num_time_samples[num_time_samples.size() - 1] - 1))
                  .transpose()
           << endl;
    }

    cout << "remove_ankle_joint_from_periodicity = "
         << remove_ankle_joint_from_periodicity << endl;
    cout << "relax_vel_periodicity_constraint = "
         << relax_vel_periodicity_constraint
         << "; (eps_vel_period = " << eps_vel_period << ")\n";
    cout << "relax_pos_periodicity_constraint = "
         << relax_pos_periodicity_constraint
         << "; (eps_pos_period = " << eps_pos_period << ")\n";

    cout << endl;
  }  // end if is_print_for_debugging
}

void trajOptGivenWeights(
    const MultibodyPlant<double>& plant,
    const MultibodyPlant<AutoDiffXd>& plant_autoDiff,
    const ReducedOrderModel& rom, InnerLoopSetting inner_loop_setting,
    Task task, const SubQpData& QPs,
    const vector<std::shared_ptr<int>>& thread_finished_vec,
    bool is_get_nominal, bool extend_model, int sample_idx, int n_rerun,
    double cost_threshold_for_update, int N_rerun, int rom_option,
    int robot_option) {
  if (robot_option == 0) {
    fiveLinkRobotTrajOpt(plant, plant_autoDiff, rom, inner_loop_setting, task,
                         QPs, is_get_nominal, extend_model, sample_idx, n_rerun,
                         cost_threshold_for_update, N_rerun, rom_option,
                         robot_option);
  } else if (robot_option == 1) {
    cassieTrajOpt(plant, plant_autoDiff, rom, inner_loop_setting, task, QPs,
                  is_get_nominal, extend_model, sample_idx, n_rerun,
                  cost_threshold_for_update, N_rerun, rom_option, robot_option);
  }

  // For multithreading purpose. Indicate this function has ended.
  *(thread_finished_vec[sample_idx]) = 1;
}

}  // namespace dairlib::goldilocks_models
