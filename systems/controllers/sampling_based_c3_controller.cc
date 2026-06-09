#include "sampling_based_c3_controller.h"

#include <ctime>
#include <iostream>
#include <thread>
#include <utility>

#include <Eigen/Dense>
#include <omp.h>

#include "c3/core/c3_miqp.h"
#include "c3/core/c3_plus.h"
#include "c3/core/c3_qp.h"
#include "c3/core/lcs.h"
#include "c3/core/traj_eval.h"
#include "c3/multibody/lcs_factory.h"
#include "common/quaternion_error_hessian.h"
#include "dairlib/lcmt_radio_out.hpp"
#include "examples/sampling_c3/generate_samples.h"
#include "examples/sampling_c3/reposition.h"
#include "multibody/multibody_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {

using c3::C3;
using c3::C3MIQP;
using c3::C3Plus;
using c3::C3QP;
using c3::LCS;
using c3::multibody::LCSContactDescription;
using c3::multibody::LCSFactory;
using c3::systems::C3Output;
using c3::traj_eval::TrajectoryEvaluator;
using drake::AutoDiffXd;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using Eigen::AngleAxisd;
using Eigen::Array;
using Eigen::Dynamic;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Quaterniond;
using Eigen::RowVectorXd;
using Eigen::seqN;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::VectorXf;
using std::vector;
using systems::TimestampedVector;

namespace systems {

SamplingC3Controller::SamplingC3Controller(
    MultibodyPlant<double>& plant, Context<double>* context,
    MultibodyPlant<AutoDiffXd>& plant_ad, Context<AutoDiffXd>* context_ad,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms,
    SamplingC3ControllerParams controller_params, bool verbose)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      controller_params_(std::move(controller_params)),
      sampling_c3_options_(controller_params_.sampling_c3_options),
      sampling_params_(controller_params_.sampling_params),
      reposition_params_(controller_params_.reposition_params),
      progress_params_(controller_params_.progress_params),
      goal_params_(controller_params_.goal_params),
      adaptive_ee_tilt_(controller_params_.include_end_effector_orientation),
      G_(vector<MatrixXd>(sampling_c3_options_.N, sampling_c3_options_.G)),
      U_(vector<MatrixXd>(sampling_c3_options_.N, sampling_c3_options_.U)),
      N_(sampling_c3_options_.N),
      verbose_(verbose) {
  this->set_name("sampling_c3_controller");

  // Build C3Options from SamplingC3Options.
  C3Options c3_options =
      sampling_c3_options_.GetC3Options(crossed_cost_switching_threshold_);

  DRAKE_DEMAND(sampling_c3_options_.lcs_dt_resolution > 0);
  dt_ =
      sampling_c3_options_.planning_dt_position;  // Initialize dt_ to position
                                                  // mode's dt by default.

  // Initialize Q_ and R_ to proper size.  Values don't matter because the
  // values get rewritten at the beginning of every control loop.
  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options.Q);
    R_.push_back(discount_factor * c3_options.R);
    discount_factor *= c3_options.gamma;
  }
  Q_.push_back(discount_factor * c3_options.Q);

  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);
  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  if (verbose_) {
    std::cout << "resolution: " << sampling_c3_options_.lcs_dt_resolution
              << std::endl;
    std::cout << "n_q_" << n_q_ << std::endl;
    std::cout << "n_v_" << n_v_ << std::endl;
    std::cout << "n_u_" << n_u_ << std::endl;
    std::cout << "n_x_" << n_x_ << std::endl;
    std::cout << "Q Rows: " << Q_[0].rows() << std::endl;
    std::cout << "Q Cols: " << Q_[0].cols() << std::endl;
    std::cout << "R Rows: " << R_[0].rows() << std::endl;
    std::cout << "R Cols: " << R_[0].cols() << std::endl;
    std::cout << "G Rows: " << G_[0].rows() << std::endl;
    std::cout << "G Cols: " << G_[0].cols() << std::endl;
    std::cout << "U Rows: " << U_[0].rows() << std::endl;
    std::cout << "U Cols: " << U_[0].cols() << std::endl;
  }
  solve_time_filter_constant_ = sampling_c3_options_.solve_time_filter_alpha;

  DRAKE_DEMAND(sampling_c3_options_.num_contacts.has_value());
  DRAKE_DEMAND(
      sampling_c3_options_.num_friction_directions_per_contact.has_value());
  n_lambda_ = LCSFactory::GetNumContactVariables(
      c3::multibody::GetContactModelMap().at(
          controller_params_.sampling_c3_options.contact_model),
      sampling_c3_options_.num_contacts.value(),
      sampling_c3_options_.num_friction_directions_per_contact.value());

  // Placeholder LCS will have correct size as it's already determined by the
  // contact model.
  auto lcs_placeholder =
      LCS::CreatePlaceholderLCS(n_x_, n_u_, n_lambda_, sampling_c3_options_.N,
                                sampling_c3_options_.planning_dt_position);

  auto x_desired_placeholder = vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));

  if (sampling_c3_options_.projection_type == "MIQP") {
    c3_curr_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3MIQP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  } else if (sampling_c3_options_.projection_type == "QP") {
    c3_curr_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
  } else if (sampling_c3_options_.projection_type == "C3+") {
    c3_curr_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3Plus>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  n_z_ = c3_curr_plan_->GetZSize();

  c3_curr_plan_->SetSolverOptions(solver_options_);
  c3_best_plan_->SetSolverOptions(solver_options_);
  c3_buffer_plan_->SetSolverOptions(solver_options_);

  // Set actor bounds.
  if (!sampling_c3_options_.include_walls) {
    for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
      RowVectorXd A = VectorXd::Zero(n_x_);
      A.segment(0, 3) = sampling_c3_options_.workspace_limits[i].segment(0, 3);
      // TODO @bibit: For the T example, the z constraint is an equality
      // constraint. This will be reflected in the params but need to make sure
      // to put a comment here when the T example is added. The fourth parameter
      // decides which optimization variable the constraint is applied to. 1 =
      // x, 2 = u, 3 = lambda.
      c3_curr_plan_->AddLinearConstraint(
          A, sampling_c3_options_.workspace_limits[i][3],
          sampling_c3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
      c3_best_plan_->AddLinearConstraint(
          A, sampling_c3_options_.workspace_limits[i][3],
          sampling_c3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
      c3_buffer_plan_->AddLinearConstraint(
          A, sampling_c3_options_.workspace_limits[i][3],
          sampling_c3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
    }
  }

  for (int i : vector<int>({0, 1})) {
    RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_horizontal_limits[0],
        sampling_c3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_best_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_horizontal_limits[0],
        sampling_c3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_buffer_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_horizontal_limits[0],
        sampling_c3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
  }
  for (int i : vector<int>({2})) {
    RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_vertical_limits[0],
        sampling_c3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_best_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_vertical_limits[0],
        sampling_c3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_buffer_plan_->AddLinearConstraint(
        A, sampling_c3_options_.u_vertical_limits[0],
        sampling_c3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
  }

  // Compute fixed quantities used for computing the adaptive EE tilt angle, if
  // enabled.
  workspace_center_[0] = (sampling_c3_options_.workspace_limits[0][3] +
                          sampling_c3_options_.workspace_limits[0][4]) /
                         2;
  workspace_center_[1] = (sampling_c3_options_.workspace_limits[1][3] +
                          sampling_c3_options_.workspace_limits[1][4]) /
                         2;
  Vector2d max_radius(
      sampling_c3_options_.workspace_limits[0][4] - workspace_center_[0],
      sampling_c3_options_.workspace_limits[1][4] - workspace_center_[1]);
  max_ee_dist_from_workspace_center_ = max_radius.norm();

  // Input ports.
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();
  final_target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_final_des", n_x_).get_index();

  // Output ports.
  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.z_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.w_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  auto lcs_contact_descriptions = vector<LCSContactDescription>();

  // Since the num_additional_samples_repos means the additional samples
  // to generate in addition to the prev_repositioning_target_, add 1.
  // Additionally add 1 to C3 if considering samples from the buffer.
  int from_buffer = 0;
  if (sampling_params_.consider_best_buffer_sample_when_leaving_c3) {
    from_buffer = 1;
  }
  max_num_samples_ =
      std::max(sampling_params_.num_additional_samples_repos + 1,
               sampling_params_.num_additional_samples_c3 + from_buffer);
  // The +1 here is to account for the current location.
  all_sample_locations_ =
      vector<Vector3d>(max_num_samples_ + 1, Vector3d::Zero());
  LcmTrajectory lcm_traj = LcmTrajectory();

  // Current location plan output ports.
  // This output port is being kept so it can go into a C3outputSender which is
  // what we use to grab downstream forces for visualization.
  c3_solution_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan", c3_solution,
              &SamplingC3Controller::OutputC3SolutionCurrPlan)
          .get_index();
  c3_solution_curr_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionCurrPlanActor)
          .get_index();
  c3_solution_curr_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionCurrPlanObject)
          .get_index();
  c3_intermediates_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_intermediates_curr_plan", c3_intermediates,
              &SamplingC3Controller::OutputC3IntermediatesCurrPlan)
          .get_index();
  lcs_contact_jacobian_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "J_lcs_curr_plan, p_lcs_curr_plan", lcs_contact_descriptions,
              &SamplingC3Controller::OutputLCSContactJacobianCurrPlan)
          .get_index();

  // Best sample plan output ports.
  // This output port is being kept so it can go into a C3outputSender which is
  // what we use to grab downstream forces for visualization.
  c3_solution_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan", c3_solution,
              &SamplingC3Controller::OutputC3SolutionBestPlan)
          .get_index();
  c3_solution_best_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionBestPlanActor)
          .get_index();
  c3_solution_best_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionBestPlanObject)
          .get_index();
  c3_intermediates_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_intermediates_best_plan", c3_intermediates,
              &SamplingC3Controller::OutputC3IntermediatesBestPlan)
          .get_index();
  lcs_contact_jacobian_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "J_lcs_best_plan, p_lcs_best_plan", lcs_contact_descriptions,
              &SamplingC3Controller::OutputLCSContactJacobianBestPlan)
          .get_index();

  // Execution trajectory output ports.
  c3_traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_traj_execute_actor", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3TrajExecuteActor)
          .get_index();
  repos_traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "repos_traj_execute_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputReposTrajExecuteActor)
          .get_index();
  traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "traj_execute_actor", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputTrajExecuteActor)
          .get_index();
  is_c3_mode_port_ =
      this->DeclareAbstractOutputPort("is_c3_mode",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &SamplingC3Controller::OutputIsC3Mode)
          .get_index();

  // Output ports for dynamically feasible plans used for cost computation and
  // visualization.
  dynamically_feasible_curr_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_curr_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanActor)
          .get_index();
  dynamically_feasible_curr_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_curr_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanObject)
          .get_index();
  dynamically_feasible_best_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_best_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleBestPlanActor)
          .get_index();
  dynamically_feasible_best_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_best_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleBestPlanObject)
          .get_index();

  // Sample location related output ports.
  // This port will output all samples except the current location.
  // all_sample_locations_port_ does not include the current location. So
  // index 0 is the first sample.
  all_sample_locations_port_ =
      this->DeclareAbstractOutputPort(
              "all_sample_locations", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputAllSampleLocations)
          .get_index();
  // all_sample_costs_port_ does include the current location. So index 0 is
  // the current location cost.
  all_sample_costs_port_ =
      this->DeclareAbstractOutputPort(
              "all_sample_costs", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputAllSampleCosts)
          .get_index();

  // A debug output port to publish information about the internals of the
  // sampling-based controller.
  debug_lcmt_port_ =
      this->DeclareAbstractOutputPort("sampling_c3_debug",
                                      dairlib::lcmt_sampling_c3_debug(),
                                      &SamplingC3Controller::OutputDebug)
          .get_index();

  // Sample buffers.
  ResetSampleBuffers();
  sample_buffer_configurations_port_ =
      this->DeclareAbstractOutputPort(
              "sample_buffer_configurations", sample_buffer_,
              &SamplingC3Controller::OutputSampleBufferConfigurations)
          .get_index();
  sample_buffer_costs_port_ =
      this->DeclareAbstractOutputPort(
              "sample_buffer_costs", sample_costs_buffer_,
              &SamplingC3Controller::OutputSampleBufferCosts)
          .get_index();
  unsuccessful_sample_buffer_configurations_port_ =
      this->DeclareAbstractOutputPort(
              "unsuccessful_sample_buffer_configurations",
              unsuccessful_sample_buffer_,
              &SamplingC3Controller::
                  OutputUnsuccessfulSampleBufferConfigurations)
          .get_index();
  unsuccessful_sample_buffer_costs_port_ =
      this->DeclareAbstractOutputPort(
              "unsuccessful_sample_buffer_costs",
              unsuccessful_sample_costs_buffer_,
              &SamplingC3Controller::OutputUnsuccessfulSampleBufferCosts)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_curr_plan_ = VectorXd::Zero(n_x_);
  x_from_last_control_loop_ = VectorXd::Zero(n_x_);
  x_pred_from_last_control_loop_ = VectorXd::Zero(n_x_);
  x_final_target_ = VectorXd::Zero(n_x_);

  ResetProgressMetrics();

  DeclareForcedDiscreteUpdateEvent(&SamplingC3Controller::ComputePlan);

  // Set Kp and Kd vectors for cost computation.  These are only used for
  // certain cost computation types.
  Kp_for_cost_ = VectorXd::Zero(n_x_);
  Kp_for_cost_(0) = sampling_c3_options_.Kp_for_ee_pd_rollout[0];
  Kp_for_cost_(1) = sampling_c3_options_.Kp_for_ee_pd_rollout[1];
  Kp_for_cost_(2) = sampling_c3_options_.Kp_for_ee_pd_rollout[2];
  Kd_for_cost_ = VectorXd::Zero(n_x_);
  Kd_for_cost_(n_q_ + 0) = sampling_c3_options_.Kd_for_ee_pd_rollout[0];
  Kd_for_cost_(n_q_ + 1) = sampling_c3_options_.Kd_for_ee_pd_rollout[1];
  Kd_for_cost_(n_q_ + 2) = sampling_c3_options_.Kd_for_ee_pd_rollout[2];

  // Set parallelization settings.
  omp_set_dynamic(0);  // Explicitly disable dynamic teams.
  omp_set_nested(1);   // Enable nested threading.
  if (sampling_c3_options_.num_outer_threads == 0) {
    // Interpret setting number of threads to zero as a request to use all
    // machine's threads.
    num_threads_to_use_ = omp_get_max_threads();
  } else {
    num_threads_to_use_ = sampling_c3_options_.num_outer_threads;
  }

  if (verbose_) {
    std::cout << "Initial filtered_solve_time_: " << filtered_solve_time_
              << std::endl;
  }

  // Below code loads in the mesh and enumerates triangular faces.
  if (sampling_params_.sampling_strategy == SamplingStrategy::kMeshNormal ||
      sampling_params_.sampling_strategy ==
          SamplingStrategy::kMeshNormalMultiObject) {
    vector<std::string> mesh_paths;
    for (std::string base_name : controller_params_.base_names) {
      std::string path =
          "examples/sampling_c3/urdf/" + base_name + "/" + base_name + ".obj";
      mesh_paths.push_back(path);
    }
    if (mesh_paths.empty()) {
      throw std::runtime_error(
          "SamplingC3Controller: no mesh files found in SceneGraph");
    }

    // N OBJECTS
    // Store faces and bins for each object

    for (const std::string& mesh_path : mesh_paths) {
      drake::geometry::TriangleSurfaceMesh<double>* mesh =
          new drake::geometry::TriangleSurfaceMesh<double>(
              drake::geometry::ReadObjToTriangleSurfaceMesh(mesh_path, 1.0));

      const auto& vertices = mesh->vertices();
      int num_tri = mesh->num_triangles();

      vector<Face> object_faces;
      vector<double> object_bins;
      object_bins.push_back(0.0);

      double cumulative_area = 0.0;

      for (int i = 0; i < num_tri; ++i) {
        auto tri = mesh->triangles()[i];
        Vector3d v0 = vertices[tri.vertex(0)];
        Vector3d v1 = vertices[tri.vertex(1)];
        Vector3d v2 = vertices[tri.vertex(2)];
        Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();

        // reject faces with normal vectors that are too vertical, with some
        // buffer to account for inaccurate object tracking
        double z_accept =
            std::pow(sampling_params_.buffer_distance, 2) -
            std::pow(sampling_params_.sample_projection_clearance, 2);
        if (std::pow(std::abs(normal[2]), 2) < z_accept + 0.04) {
          double area = 0.5 * (v1 - v0).cross(v2 - v0).norm();
          object_faces.push_back({area, normal, {v0, v1, v2}});
          cumulative_area += area;
          object_bins.push_back(cumulative_area);
        }
      }

      if (object_faces.empty()) {
        throw std::runtime_error("No valid faces found in " + mesh_path);
      }

      faces_per_object_.push_back(std::move(object_faces));
      face_bins_per_object_.push_back(std::move(object_bins));
      total_area_per_object_.push_back(cumulative_area);
    }
  }
}

// This function relies on the previously computed z_fin from Solve.
std::pair<double, vector<VectorXd>> SamplingC3Controller::CalcCost(
    C3CostComputationType cost_type, const LCS& lcs_for_cost,
    const C3::CostMatrices& cost_mats, const std::shared_ptr<C3>& c3_object,
    const bool& force_tracking_disabled, int num_objects,
    const bool& print_cost_breakdown) const {
  // Extract needed information from the C3 object.
  const LCS lcs_for_plan = c3_object->GetLCS();
  vector<VectorXd> x_desired = c3_object->GetDesiredState();
  vector<VectorXd> x_plan = c3_object->GetStateSolution();
  vector<VectorXd> u_plan = c3_object->GetInputSolution();
  vector<VectorXd> lambda_plan = c3_object->GetForceSolution();

  // TODO @bibit: The original CalcCost extracts the x and u trajectories from
  // the C3 object's z_sol_ vector, which can differ from the C3 object's
  // x_sol_ and u_sol_ vectors if end_on_qp_step is false.  This may be
  // considered a bug in C3, but for now, extract the same trajectories to
  // maintain functionality.  If C3 is fixed so that x_sol_ and u_sol_ (and
  // lambda_sol_) always reflect the trajectories corresponding to z_sol_, then
  // the following 5 lines can be removed since x_plan and u_plan will already
  // be correct from the above getters.
  vector<VectorXd> z_plan = c3_object->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    x_plan[i] = z_plan[i].segment(0, n_x_);
    lambda_plan[i] = z_plan[i].segment(n_x_, n_lambda_);
    u_plan[i] = z_plan[i].segment(n_x_ + n_lambda_, n_u_);
  }
  DRAKE_THROW_UNLESS(z_plan.size() == N_);
  DRAKE_THROW_UNLESS(x_plan.size() == N_);
  DRAKE_THROW_UNLESS(u_plan.size() == N_);

  // The x_plan from the C3 object does not include the x_N state, so add it in
  // using the LCS rollout from the last x, u, and lambda.
  MatrixXd A_N = lcs_for_plan.A().back();
  MatrixXd B_N = lcs_for_plan.B().back();
  MatrixXd D_N = lcs_for_plan.D().back();
  VectorXd d_N = lcs_for_plan.d().back();
  x_plan.push_back(A_N * x_plan.back() + B_N * u_plan.back() +
                   D_N * lambda_plan.back() + d_N);

  // Initialize the cost-driving trajectories to match the C3 plan.
  vector<VectorXd> XX = x_plan;
  vector<VectorXd> UU = u_plan;

  // Declare the matrices to use for cost computation.
  vector<MatrixXd> Q_cost = cost_mats.Q;
  vector<MatrixXd> R_cost = cost_mats.R;

  // Set a few more variables necessary for some of the cost types.
  const int ee_vel_index = 3 + 7 * num_objects;
  auto simulate_config = c3::LCSSimulateConfig();
  simulate_config.regularized = true;
  simulate_config.min_exp = -8;

  // Compute the states and controls to use for cost computation, and change the
  // cost matrices if necessary.
  if (cost_type == C3CostComputationType::kSimLCS) {
    // Simulate the LCS from initial condition using the C3 plan's controls.
    XX = TrajectoryEvaluator::SimulateLCSOverTrajectory(
        x_plan[0], u_plan, lcs_for_plan, lcs_for_cost, simulate_config);

  } else if (cost_type == C3CostComputationType::kUseC3Plan) {
    // No need to do anything here.

  } else if (cost_type == C3CostComputationType::kSimLCSReplaceC3EEPlan) {
    // Simulate the LCS from initial condition using the C3 plan's controls.
    vector<VectorXd> XX_sim = TrajectoryEvaluator::SimulateLCSOverTrajectory(
        x_plan[0], u_plan, lcs_for_plan, lcs_for_cost, simulate_config);

    // Use the simulated object trajectories but the planned robot trajectory.
    for (int i = 0; i < N_ + 1; i++) {
      XX[i].segment(3, 7 * num_objects) = XX_sim[i].segment(3, 7 * num_objects);
      XX[i].segment(ee_vel_index + 3, 6 * num_objects) =
          XX_sim[i].segment(ee_vel_index + 3, 6 * num_objects);
    }

  } else if (cost_type == C3CostComputationType::kSimImpedance) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    XX = XX_sim;
    UU = UU_sim;

  } else if (cost_type == C3CostComputationType::kSimImpedanceReplaceC3EEPlan) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    UU = UU_sim;

    // Use the simulated object trajectories but the planned robot trajectory.
    for (int i = 0; i < N_ + 1; i++) {
      XX[i].segment(3, 7 * num_objects) = XX_sim[i].segment(3, 7 * num_objects);
      XX[i].segment(ee_vel_index + 3, 6 * num_objects) =
          XX_sim[i].segment(ee_vel_index + 3, 6 * num_objects);
    }

  } else if (cost_type == C3CostComputationType::kSimImpedanceObjectCostOnly) {
    // Simulate PD with feedforward control using the C3 plan's states and
    // controls from the initial condition.
    auto [XX_sim, UU_sim] = TrajectoryEvaluator::SimulatePDControlWithLCS(
        x_plan, UU, Kp_for_cost_, Kd_for_cost_, lcs_for_plan, lcs_for_cost,
        !force_tracking_disabled, simulate_config);
    XX = XX_sim;
    UU = UU_sim;

    // Set R and the robot portion of the Q matrix to zero so that only the
    // object state errors contribute to cost.
    for (int i = 0; i < N_ + 1; i++) {
      Q_cost[i].block(0, 0, 3, 3) *= 0.0;
      Q_cost[i].block(3 + 7 * num_objects, 3 + 7 * num_objects, 3, 3) *= 0.0;
      if (i < N_) {
        R_cost[i] *= 0.0;
      }
    }
  }

  // Compute the cost.
  double cost = TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
      XX, x_desired, Q_cost, UU, R_cost);

  if (print_cost_breakdown) {
    std::cout << "===== NEW COST BREAKDOWN =====" << std::endl;
    // Errors
    MatrixXd Q_identity = MatrixXd::Identity(n_x_, n_x_);
    double error_contrib_ee_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(0, 3, XX, x_desired,
                                                            Q_identity);
    double error_contrib_ee_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            ee_vel_index, ee_vel_index + 3, XX, x_desired, Q_identity);

    double error_contrib_obj_orientation = 0.0;
    double error_contrib_obj_pos = 0.0;
    double error_contrib_obj_ang_vel = 0.0;
    double error_contrib_obj_vel = 0.0;
    for (int obj_idx = 0; obj_idx < num_objects; obj_idx++) {
      error_contrib_obj_orientation +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              3 + 7 * obj_idx, 3 + 7 * obj_idx + 4, XX, x_desired, Q_identity);
      error_contrib_obj_pos +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              3 + 7 * obj_idx + 4, 3 + 7 * obj_idx + 7, XX, x_desired,
              Q_identity);
      error_contrib_obj_ang_vel +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              ee_vel_index + 3 + 6 * obj_idx,
              ee_vel_index + 3 + 6 * obj_idx + 3, XX, x_desired, Q_identity);
      error_contrib_obj_vel +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              ee_vel_index + 3 + 6 * obj_idx + 3,
              ee_vel_index + 3 + 6 * obj_idx + 6, XX, x_desired, Q_identity);
    }

    // Costs
    double cost_contrib_ee_pos =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(0, 3, XX, x_desired,
                                                            Q_cost);
    double cost_contrib_ee_vel =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
            ee_vel_index, ee_vel_index + 3, XX, x_desired, Q_cost);
    double cost_contrib_u =
        TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(UU, R_cost);

    double cost_contrib_obj_orientation = 0.0;
    double cost_contrib_obj_pos = 0.0;
    double cost_contrib_obj_ang_vel = 0.0;
    double cost_contrib_obj_vel = 0.0;
    for (int obj_idx = 0; obj_idx < num_objects; obj_idx++) {
      cost_contrib_obj_orientation +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              3 + 7 * obj_idx, 3 + 7 * obj_idx + 4, XX, x_desired, Q_cost);
      cost_contrib_obj_pos +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              3 + 7 * obj_idx + 4, 3 + 7 * obj_idx + 7, XX, x_desired, Q_cost);
      cost_contrib_obj_ang_vel +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              ee_vel_index + 3 + 6 * obj_idx,
              ee_vel_index + 3 + 6 * obj_idx + 3, XX, x_desired, Q_cost);
      cost_contrib_obj_vel +=
          TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              ee_vel_index + 3 + 6 * obj_idx + 3,
              ee_vel_index + 3 + 6 * obj_idx + 6, XX, x_desired, Q_cost);
    }

    std::cout << "Error breakdown" << std::endl;
    std::cout << "\t total error contribution from x_ee: "
              << error_contrib_ee_pos << std::endl;
    std::cout << "\t total error contribution from q_obj: "
              << error_contrib_obj_orientation << std::endl;
    std::cout << "\t total error contribution from x_obj: "
              << error_contrib_obj_pos << std::endl;
    std::cout << "\t total error contribution from v_ee: "
              << error_contrib_ee_vel << std::endl;
    std::cout << "\t total error contribution from w_obj: "
              << error_contrib_obj_ang_vel << std::endl;
    std::cout << "\t total error contribution from v_obj: "
              << error_contrib_obj_vel << std::endl;

    std::cout << "\nCOST BREAKDOWN" << std::endl;
    std::cout << "\t total cost contribution from x_ee: " << cost_contrib_ee_pos
              << std::endl;
    std::cout << "\t total cost contribution from q_obj: "
              << cost_contrib_obj_orientation << std::endl;
    std::cout << "\t total cost contribution from x_obj: "
              << cost_contrib_obj_pos << std::endl;
    std::cout << "\t total cost contribution from v_ee: " << cost_contrib_ee_vel
              << std::endl;
    std::cout << "\t total cost contribution from w_obj: "
              << cost_contrib_obj_ang_vel << std::endl;
    std::cout << "\t total cost contribution from v_obj: "
              << cost_contrib_obj_vel << std::endl;
    std::cout << "\t total cost contribution from u: " << cost_contrib_u
              << std::endl;

    std::cout << "\t total cost is: " << cost << std::endl;
    std::cout << "\t total cost object terms only is : "
              << cost_contrib_obj_pos + cost_contrib_obj_orientation +
                     cost_contrib_obj_vel + cost_contrib_obj_ang_vel
              << std::endl;
    std::cout << "\n\n";
  }

  std::pair<double, vector<VectorXd>> ret(cost, XX);
  return ret;
}

drake::systems::EventStatus SamplingC3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();
  auto section_start = start;
  std::vector<std::pair<std::string, double>> timing_breakdown_ms;
  auto record_timing = [&](const std::string& section_name) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                            now - section_start)
                            .count() /
                        1e3;
    timing_breakdown_ms.emplace_back(section_name, elapsed_ms);
    section_start = now;
  };

  // Evaluate input ports.
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  // Not sure why x_lcs_des is a vector while lcs_x_curr is a timestamped
  // vector.
  const BasicVector<double>& x_lcs_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const BasicVector<double>& x_lcs_final_des =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   final_target_input_port_);
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  // Store the current LCS state.
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();

  record_timing("Input evaluation");
  ee_position_curr_ = x_lcs_curr.segment(0, 3);
  if (verbose_) {
    std::cout << "x_lcs_curr: " << x_lcs_curr.transpose() << std::endl;
    std::cout << "x_lcs_des: " << x_lcs_des.get_value().transpose()
              << std::endl;
    std::cout << "x_lcs_final_des: " << x_lcs_final_des.get_value().transpose()
              << std::endl;
    std::cout << "x_pred_curr_plan_: " << x_pred_curr_plan_.transpose()
              << std::endl;
  }

  // Use a predicted EE state, if desired.
  bool is_teleop = radio_out->channel[14];
  ResolvePredictedEEState(is_teleop, x_lcs_curr);

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x_curr->get_timestamp();

  // Check for workspace limit violations; if any, the controller stops.
  CheckForWorkspaceLimitViolations(lcs_x_curr);

  // Compute the current position and orientation errors.
  current_position_error_ = 0;
  current_orientation_error_ = 0;

  for (int i = 0; i < controller_params_.num_objects; i++) {
    double error = (x_lcs_curr.segment(7 + 7 * i, 3) -
                    x_lcs_final_des.get_value().segment(7 + 7 * i, 3))
                       .norm();
    current_position_error_ += error;

    Quaterniond curr_quat(x_lcs_curr[3 + 7 * i], x_lcs_curr[4 + 7 * i],
                          x_lcs_curr[5 + 7 * i], x_lcs_curr[6 + 7 * i]);
    Quaterniond des_quat(x_lcs_final_des.get_value()[3 + 7 * i],
                         x_lcs_final_des.get_value()[4 + 7 * i],
                         x_lcs_final_des.get_value()[5 + 7 * i],
                         x_lcs_final_des.get_value()[6 + 7 * i]);
    AngleAxisd angle_axis_diff(des_quat * curr_quat.inverse());
    current_orientation_error_ += angle_axis_diff.angle();
  }

  // Detect if the final target has changed, in which case return to caring only
  // about position until the switching threshold has been crossed again.
  // Exclude the EE goal from the comparison, since that always changes to be
  // above the current object location.
  if (!x_final_target_.segment(3, n_x_ - 3)
           .isApprox(x_lcs_final_des.value().segment(3, n_x_ - 3), 1e-5)) {
    std::cout << "Detected goal change!" << std::endl;
    if (verbose_) {
      std::cout << "  Last goal: " << x_final_target_.transpose() << std::endl;
      std::cout << "  New goal:  " << x_lcs_final_des.value().transpose()
                << std::endl;
      std::cout << "  --> Error:  "
                << (x_final_target_.segment(3, n_x_ - 3) -
                    x_lcs_final_des.value().segment(3, n_x_ - 3))
                       .norm()
                << std::endl;
    }
    crossed_cost_switching_threshold_ = false;
    dt_ =
        sampling_c3_options_.planning_dt_position;  // Always set dt_ according
                                                    // to pose or position mode.
    x_final_target_ = x_lcs_final_des.value();
    // is_doing_c3_ = false;
    detected_goal_changes_++;

    // Reset the sample buffers now that the costs have changed.
    ResetSampleBuffers();
  }

  // If the object is close to desired XY location, track its full pose.
  if (!crossed_cost_switching_threshold_) {
    double pose_diff = 0;
    for (int i = 0; i < controller_params_.num_objects; i++) {
      pose_diff += (x_lcs_curr.segment(7 + 7 * i, 2) -
                    x_lcs_final_des.value().segment(7 + 7 * i, 2))
                       .norm();
    }
    if (pose_diff < progress_params_.cost_switching_threshold_distance *
                        controller_params_.num_objects) {
      crossed_cost_switching_threshold_ = true;
      dt_ = sampling_c3_options_.planning_dt_pose;  // Always set dt_ according
                                                    // to pose or position mode.
      std::cout << "Crossed cost switching threshold." << std::endl;

      // Reset the sample buffers and metrics now that the costs have changed.
      ResetSampleBuffers();
      if (is_doing_c3_) {
        ResetProgressMetrics();
      }
    }
  }

  record_timing("State prep, safety checks, and goal/mode-threshold updates");

  // Build C3Options from SamplingC3Options based on the
  // crossed_cost_switching_threshold_ flag.
  C3Options c3_options =
      sampling_c3_options_.GetC3Options(crossed_cost_switching_threshold_);
  LCSFactoryOptions lcs_factory_options =
      sampling_c3_options_.GetLCSFactoryOptions(
          crossed_cost_switching_threshold_);

  // Update the cost matrices:  Q_, R_, G_, and U_.
  UpdateCostMatrices(x_lcs_curr, x_lcs_des, c3_options);

  record_timing("Options and cost matrix update");

  // Detect if all goals have been reached.
  vector<bool> object_on_target;
  bool all_reached = true;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    double object_position_error = (x_lcs_curr.segment(7 + 7 * i, 2) -
                                    x_lcs_des.get_value().segment(7 + 7 * i, 2))
                                       .norm();
    Quaterniond q_des(x_lcs_des.get_value().segment<4>(3 + 7 * i));
    Quaterniond q_curr(x_lcs_curr.segment<4>(3 + 7 * i));

    AngleAxisd angle_axis_diff(q_des * q_curr.inverse());
    double object_angular_error = angle_axis_diff.angle();

    object_on_target.push_back(
        ((crossed_cost_switching_threshold_) &&
         (object_position_error < goal_params_.position_success_threshold) &&
         (object_angular_error < goal_params_.orientation_success_threshold)) ||
        ((!crossed_cost_switching_threshold_) &&
         (object_position_error < goal_params_.position_success_threshold)));
    all_reached = all_reached && object_on_target[i];
  }

  record_timing("Goal achievement detection");

  // Generate states, differing from the current state only by EE sample
  // locations.
  // Used fixed samples if fixed goal and all objects on target.
  vector<VectorXd> candidate_states;
  if (achieved_fixed_goal_ ||
      (all_reached && goal_params_.goal_mode == GoalMode::kFixedGoal)) {
    achieved_fixed_goal_ = true;
    int num_samples = is_doing_c3_
                          ? sampling_params_.num_additional_samples_c3
                          : sampling_params_.num_additional_samples_repos;
    for (int i = 0; i < num_samples; i++) {
      candidate_states.push_back(x_lcs_curr);
      candidate_states[i].head(3) << 0.3, 0.4, 0.1;
    }
  }
  // Generate new samples according to sampling strategy.
  else {
    candidate_states = GenerateSampleStates(
        n_q_, n_v_, n_u_, x_lcs_curr, is_doing_c3_, sampling_params_,
        sampling_c3_options_, plant_, context_, plant_ad_, context_ad_,
        contact_pairs_, faces_, face_bins_, faces_per_object_,
        face_bins_per_object_, total_area_per_object_, object_on_target,
        unsuccessful_sample_buffer_);
  }

  // Ensure prev_repositing_target_ is not inside the object
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<double>>(*context_);

  const auto& results = query_object.ComputeSignedDistanceToPoint(
      prev_repositioning_target_.segment(0, 3));
  bool in_collision = false;
  // Index 0 is the robot; index 1 is the ground, indices 2+ are the object(s)
  // and walls (if any).  It's ok to detect collision with the walls; if wanted
  // to only check collision with the objects, would iterate up to
  // results.size() - 0 or 3 or 4, depending on if 0, 3, or 4 walls are included
  // in the plant.
  for (int i = 2; i < results.size(); i++) {
    if (results[i].distance <= sampling_params_.sample_projection_clearance) {
      in_collision = true;
      break;
    }
  }

  // Add the previous best repositioning target to the candidate states at index
  // 1 if in C3 mode and if the previous target is not in collision. (Index 0
  // will become the current state.)
  if (!is_doing_c3_ && !in_collision) {
    VectorXd repositioning_target_state = x_lcs_curr;
    repositioning_target_state.head(3) = prev_repositioning_target_;
    candidate_states.insert(candidate_states.begin(),
                            repositioning_target_state);
  }
  // Insert the current location at the beginning of the candidate states.
  candidate_states.insert(candidate_states.begin(), x_lcs_curr);
  int num_total_samples = candidate_states.size();

  if (verbose_) {
    std::cout << "num_total_samples: " << num_total_samples << std::endl;
  }

  // Update the set of sample locations under consideration.
  all_sample_locations_.clear();
  for (int i = 0; i < num_total_samples; i++) {
    all_sample_locations_.push_back(candidate_states[i].head(3));
  }
  // Make LCS objects for each sample.
  auto lcs_pair = SamplingC3Controller::CreateLCSObjectsForSamples(
      candidate_states, x_lcs_curr, lcs_factory_options);
  vector<LCS> lcs_candidates = lcs_pair.first;
  vector<LCS> lcs_candidates_for_cost = lcs_pair.second;

  record_timing("Sample generation and LCS construction");

  // Prepare variables that will get used or filled in by parallelization.
  all_sample_costs_ = vector<double>(num_total_samples, -1);
  all_sample_dynamically_feasible_plans_ = vector<vector<VectorXd>>(
      num_total_samples, vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_)));
  vector<std::shared_ptr<C3>> c3_objects(num_total_samples, nullptr);
  bool force_tracking_disabled = radio_out->channel[11];
  C3CostComputationType cost_type = progress_params_.cost_type;
  if (!crossed_cost_switching_threshold_) {
    cost_type = progress_params_.cost_type_position;
  }

  // Parallelize over computing C3 costs for each sample.
  auto c3_start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for num_threads(num_threads_to_use_)
  for (int i = 0; i < num_total_samples; i++) {
    auto parallel_start = std::chrono::high_resolution_clock::now();
    bool print_cost_breakdown =
        radio_out->channel[7] && (i == SampleIndex::kCurrentLocation);

    auto after_radio = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                            after_radio - parallel_start)
                            .count() /
                        1e3;
    std::cout << "  " << i << ": Radio read: " << elapsed_ms << " ms"
              << std::endl;

    // Get the candidate state and its LCS representation.
    VectorXd test_state = candidate_states.at(i);
    LCS test_system = lcs_candidates.at(i);

    // Set up C3 with proper projection type and post-solve cost matrices.
    std::shared_ptr<C3> test_c3_object;
    vector<VectorXd> x_desired(N_ + 1, x_lcs_des.value());

    C3::CostMatrices c3_costmat(Q_, R_, G_, U_);
    if (sampling_c3_options_.projection_type == "MIQP") {
      test_c3_object = std::make_shared<C3MIQP>(test_system, c3_costmat,
                                                x_desired, c3_options);
    } else if (sampling_c3_options_.projection_type == "QP") {
      test_c3_object = std::make_shared<C3QP>(test_system, c3_costmat,
                                              x_desired, c3_options);
    } else if (sampling_c3_options_.projection_type == "C3+") {
      test_c3_object = std::make_shared<C3Plus>(test_system, c3_costmat,
                                                x_desired, c3_options);
    }  // Unknown projection types are rejected in the initialization.

    auto after_c3_build = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_c3_build - after_radio)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": C3 build: " << elapsed_ms << " ms"
              << std::endl;

    if (!sampling_c3_options_.include_walls) {
      // Set actor bounds.
      for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
        RowVectorXd A = VectorXd::Zero(n_x_);
        A.segment(0, 3) =
            sampling_c3_options_.workspace_limits[i].segment(0, 3);
        test_c3_object->AddLinearConstraint(
            A,
            sampling_c3_options_.workspace_limits[i][3] -
                sampling_c3_options_.workspace_margins,
            sampling_c3_options_.workspace_limits[i][4] +
                sampling_c3_options_.workspace_margins,
            c3::ConstraintVariable::STATE);
      }
      // Set object bounds
      for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
        for (int j = 0; j < controller_params_.num_objects; j++) {
          RowVectorXd A = VectorXd::Zero(n_x_);
          A.segment(7 + 7 * j, 3) =
              sampling_c3_options_.workspace_limits[i].segment(0, 3);
          test_c3_object->AddLinearConstraint(
              A,
              sampling_c3_options_.workspace_limits[i][3] -
                  sampling_c3_options_.workspace_margins,
              sampling_c3_options_.workspace_limits[i][4] +
                  sampling_c3_options_.workspace_margins,
              c3::ConstraintVariable::STATE);
        }
      }
    }

    // Add constraint on end-effector velocities
    for (int i : vector<int>({0, 1, 2})) {
      RowVectorXd A = VectorXd::Zero(n_x_);
      A(n_q_ + i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, sampling_c3_options_.ee_velocity_limits[0],
          sampling_c3_options_.ee_velocity_limits[1],
          c3::ConstraintVariable::STATE);
    }

    // Add input constraints
    for (int i : vector<int>({0, 1})) {
      RowVectorXd A = VectorXd::Zero(n_u_);
      A(i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, sampling_c3_options_.u_horizontal_limits[0],
          sampling_c3_options_.u_horizontal_limits[1],
          c3::ConstraintVariable::INPUT);
    }
    for (int i : vector<int>({2})) {
      RowVectorXd A = VectorXd::Zero(n_u_);
      A(i) = 1.0;
      test_c3_object->AddLinearConstraint(
          A, sampling_c3_options_.u_vertical_limits[0],
          sampling_c3_options_.u_vertical_limits[1],
          c3::ConstraintVariable::INPUT);
    }

    auto after_limits = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_limits - after_c3_build)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": C3 limits: " << elapsed_ms << " ms"
              << std::endl;

    // Solve C3, store resulting object and cost.
    test_c3_object->SetSolverOptions(solver_options_);
    test_c3_object->Solve(test_state);

    auto after_c3_solve = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_c3_solve - after_limits)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": C3 solve: " << elapsed_ms << " ms"
              << std::endl;

    std::pair<double, vector<VectorXd>> cost_trajectory_pair = CalcCost(
        cost_type, lcs_candidates_for_cost.at(i), c3_costmat, test_c3_object,
        force_tracking_disabled, controller_params_.num_objects,
        print_cost_breakdown || verbose_);

    auto after_calc_cost = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_calc_cost - after_c3_solve)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": C3 calc cost: " << elapsed_ms << " ms"
              << std::endl;

    double c3_cost = cost_trajectory_pair.first;
    all_sample_dynamically_feasible_plans_.at(i) = cost_trajectory_pair.second;

#pragma omp critical
    {
      c3_objects.at(i) = test_c3_object;
    }
    // Add travel cost (just looking at xy displacement, not also z).
    double xy_travel_distance =
        (test_state.head(2) - x_lcs_curr.head(2)).norm();
    all_sample_costs_[i] =
        c3_cost + progress_params_.travel_cost_per_meter * xy_travel_distance;

    // Add additional costs based on repositioning progress.
    if ((i == SampleIndex::kCurrentReposTarget) && finished_reposition_flag_) {
      all_sample_costs_[i] += progress_params_.finished_reposition_cost;
      finished_reposition_flag_ = false;
    }

    auto after_everything = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_everything - after_calc_cost)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": travel and finish: " << elapsed_ms << " ms"
              << std::endl;
    elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                     after_everything - parallel_start)
                     .count() /
                 1e3;
    std::cout << "  " << i << ": Total time: " << elapsed_ms << " ms"
              << std::endl;
  }
  auto c3_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = c3_end - c3_start;
  // End of parallelization

  record_timing("Parallel C3 solve and cost evaluation");

  // Update the sample buffer.  Do this before switching modes since 1) if in
  // repositioning mode, don't add the repositioning target over and over again,
  // and 2) since the best sample in the buffer may be the best sample overall
  // and could be considered as a repositioning target.
  MaintainSampleBuffers(x_lcs_curr);

  // Augment the considered samples with the best from the buffer, if eligible.
  AugmentSamplesWithBuffer(c3_objects);

  record_timing("Sample buffer maintenance and augmentation");

  // Set up hysteresis values based on if the cost switching threshold has been
  // crossed.
  double hyst_c3_to_repos = progress_params_.hyst_c3_to_repos;
  double hyst_repos_to_c3 = progress_params_.hyst_repos_to_c3;
  double hyst_repos_to_repos = progress_params_.hyst_repos_to_repos;
  double hyst_c3_to_repos_frac = progress_params_.hyst_c3_to_repos_frac;
  double hyst_repos_to_c3_frac = progress_params_.hyst_repos_to_c3_frac;
  double hyst_repos_to_repos_frac = progress_params_.hyst_repos_to_repos_frac;
  if (!crossed_cost_switching_threshold_) {
    hyst_c3_to_repos = progress_params_.hyst_c3_to_repos_position;
    hyst_repos_to_c3 = progress_params_.hyst_repos_to_c3_position;
    hyst_repos_to_repos = progress_params_.hyst_repos_to_repos_position;
    hyst_c3_to_repos_frac = progress_params_.hyst_c3_to_repos_frac_position;
    hyst_repos_to_c3_frac = progress_params_.hyst_repos_to_c3_frac_position;
    hyst_repos_to_repos_frac =
        progress_params_.hyst_repos_to_repos_frac_position;
  }

  // Review the cost results to determine the best sample.
  bool force_c3_mode = radio_out->channel[12];
  double best_other_cost;
  if (num_total_samples > 1) {
    vector<double> additional_sample_cost_vector =
        vector<double>(all_sample_costs_.begin() + 1, all_sample_costs_.end());
    best_other_cost = *std::min_element(additional_sample_cost_vector.begin(),
                                        additional_sample_cost_vector.end());
    vector<double>::iterator it =
        std::min_element(std::begin(additional_sample_cost_vector),
                         std::end(additional_sample_cost_vector));
    best_sample_index_ =
        (SampleIndex)(std::distance(std::begin(additional_sample_cost_vector),
                                    it) +
                      1);
  } else {
    force_c3_mode = true;
  }

  record_timing("Hysteresis setup and best-sample selection");

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i]
                << std::endl;
    }
    std::cout << "In C3 mode? " << is_doing_c3_ << std::endl;
  }

  // Determine whether to do C3 or reposition.
  mode_switch_reason_ = ModeSwitchReason::kNoSwitch;
  double curr_cost = all_sample_costs_[SampleIndex::kCurrentLocation];
  double repos_target_cost =
      all_sample_costs_[SampleIndex::kCurrentReposTarget];
  if (is_doing_c3_ == true) {  // Currently doing C3.
    pursued_target_source_ = PursuedTargetSource::kNoTarget;

    // Keep track of progress while in C3 mode.
    bool met_minimum_progress = true;  // Reset by below function.
    bool print_current_pos_and_rot_cost = radio_out->channel[6];
    KeepTrackOfC3ModeProgress(x_lcs_curr, x_lcs_final_des, met_minimum_progress,
                              print_current_pos_and_rot_cost);

    // Switch to repositioning if fixed goals have all been met.
    if (achieved_fixed_goal_) {
      is_doing_c3_ = false;
      std::cout << "All objects on target, switching to repositioning mode"
                << std::endl;
    }
    // Switch to repositioning if progress was insufficient.
    else if (!met_minimum_progress && !force_c3_mode &&
             (sampling_params_.num_additional_samples_c3 > 0)) {
      is_doing_c3_ = false;
      mode_switch_reason_ = ModeSwitchReason::kToReposUnproductive;
      std::cout << "Repositioning after not making progress in C3" << std::endl;
    }

    // Switch to repositioning if one of the other samples is better, with
    // hysteresis.
    else if (((!progress_params_.use_relative_hysteresis &&
               curr_cost > best_other_cost + hyst_c3_to_repos) ||
              (progress_params_.use_relative_hysteresis &&
               curr_cost >
                   best_other_cost + hyst_c3_to_repos_frac * curr_cost)) &&
             !force_c3_mode) {
      is_doing_c3_ = false;
      mode_switch_reason_ = ModeSwitchReason::kToReposCost;
      std::cout << "Repositioning because found good sample" << std::endl;
    }

    // Reset progress metrics if switching to repositioning.
    if (!is_doing_c3_) {
      finished_reposition_flag_ = false;
      ResetProgressMetrics();

      // Determine the source of the repositioning target.
      if (best_sample_index_ > sampling_params_.num_additional_samples_c3) {
        pursued_target_source_ = PursuedTargetSource::kFromBuffer;
        // Remove the sample from the buffer.
        sample_buffer_.row(num_in_buffer_ - 1) = VectorXd::Zero(n_q_);
        sample_costs_buffer_[num_in_buffer_ - 1] = -1;
        num_in_buffer_--;
      } else {
        pursued_target_source_ = PursuedTargetSource::kNewSample;
      }
    }
  } else {  // Currently repositioning.
    // First, apply hysteresis between repositioning targets.
    if (best_sample_index_ == SampleIndex::kCurrentReposTarget &&
        !in_collision) {
      pursued_target_source_ = PursuedTargetSource::kPrevious;
    } else if (in_collision) {
      // This means the previous repositioning target is now in penetration with
      // the object and has been rejected.  Switch to the new lowest cost
      // sample.
      std::cout << "Repos -> Repos:  Previous repositioning target in "
                   "collision; switching to new sample"
                << std::endl;
      pursued_target_source_ = PursuedTargetSource::kNewSample;
    } else {
      // This means there is a lower cost sample other than the current
      // repositioning target. If the lowest cost sample is not at least the
      // hysteresis amount better than the current repositioning target, then
      // continue pursuing the previous repositioning target.
      if ((repos_target_cost < best_other_cost + hyst_repos_to_repos &&
           !progress_params_.use_relative_hysteresis) ||
          (repos_target_cost <
               best_other_cost + hyst_repos_to_repos_frac * repos_target_cost &&
           progress_params_.use_relative_hysteresis)) {
        best_sample_index_ = SampleIndex::kCurrentReposTarget;
        best_other_cost = repos_target_cost;
        finished_reposition_flag_ = false;
        pursued_target_source_ = PursuedTargetSource::kPrevious;
      }
      // Controller will switch to pursuing a new sample from its previous
      // repositioning target only if the cost of switching to that new sample
      // (with repos_to_repos hysteresis) is less than switching to C3 from
      // current location (with repos_to_c3 hysteresis), so add the
      // repos_to_repos hysteresis value here before the comparison to the
      // current location C3 cost with repos_to_c3 hysteresis afterwards.
      else {
        pursued_target_source_ = PursuedTargetSource::kNewSample;
        if (!progress_params_.use_relative_hysteresis) {
          best_other_cost += hyst_repos_to_repos;
        } else {
          best_other_cost += hyst_repos_to_repos_frac * repos_target_cost;
        }
      }
    }

    double wall_offset = 0;

    if (sampling_c3_options_.include_walls && sampling_params_.sample_on_wall) {
      double x_min = sampling_c3_options_.workspace_limits[0][3];
      double x_max = sampling_c3_options_.workspace_limits[0][4];
      double y_min = sampling_c3_options_.workspace_limits[1][3];
      double y_max = sampling_c3_options_.workspace_limits[1][4];

      // if ee is close to wall, raise z_height to avoid hitting
      if ((x_lcs_curr[0] <= x_min + 0.05 &&
           x_lcs_curr[0] >= x_min - sampling_c3_options_.workspace_margins) ||
          (x_lcs_curr[0] >= x_max - 0.05 &&
           x_lcs_curr[0] <= x_max + sampling_c3_options_.workspace_margins) ||
          (x_lcs_curr[1] <= y_min + 0.05 &&
           x_lcs_curr[1] >= y_min - sampling_c3_options_.workspace_margins) ||
          (x_lcs_curr[1] >= y_max - 0.05 &&
           x_lcs_curr[1] <= y_max + sampling_c3_options_.workspace_margins)) {
        wall_offset = 0.01;
      }
    }

    // Switch to C3 if forced by xbox controller.
    if (force_c3_mode) {
      std::cout << "Forcing into C3 mode" << std::endl;
      is_doing_c3_ = true;
      mode_switch_reason_ = ModeSwitchReason::kToC3Xbox;
      pursued_target_source_ = PursuedTargetSource::kNoTarget;
      // Add the current state to the unsuccessful sample buffer.  It gets
      // automatically removed if the object moves beyond the buffer movement
      // thresholds.
      AddToUnsuccessfulBuffer(candidate_states[0]);
    }
    // Stay in repositioning if fixed goal is met.
    else if (achieved_fixed_goal_) {
      finished_reposition_flag_ = false;
      std::cout << "All objects at fixed goals; stay out of the way."
                << std::endl;
    }
    // Switch to C3 if the current sample is better, with hysteresis.
    else if (((!progress_params_.use_relative_hysteresis &&
               best_other_cost > curr_cost + hyst_repos_to_c3) ||
              (progress_params_.use_relative_hysteresis &&
               best_other_cost >
                   curr_cost + hyst_repos_to_c3_frac * best_other_cost)) &&
             (x_lcs_curr[2] < sampling_params_.z_height +
                                  sampling_params_.c3_min_clearance +
                                  wall_offset ||
              !sampling_params_.ee_z_close)) {
      is_doing_c3_ = true;
      finished_reposition_flag_ = false;
      if (repos_target_cost > progress_params_.finished_reposition_cost) {
        mode_switch_reason_ = ModeSwitchReason::kToC3ReachedReposTarget;
        std::cout << "Switching to C3 because reached repositioning target"
                  << std::endl;
      } else {
        mode_switch_reason_ = ModeSwitchReason::kToC3Cost;
        std::cout << "Switching to C3 because lower in cost" << std::endl;
      }
      pursued_target_source_ = PursuedTargetSource::kNoTarget;
      // Add the current state to the unsuccessful sample buffer.  It gets
      // automatically removed if the object moves beyond the buffer movement
      // thresholds.
      AddToUnsuccessfulBuffer(candidate_states[0]);
    }
  }

  record_timing("Mode-switch decision logic");

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i]
                << std::endl;
    }
    std::cout << "In C3 mode after considering switch? " << is_doing_c3_
              << std::endl;
  }
  // Update C3 objects and intermediates for current and best samples.
  c3_curr_plan_ = c3_objects.at(SampleIndex::kCurrentLocation);
  c3_best_plan_ = c3_objects.at(best_sample_index_);

  // TODO If doing warmstarting, will need to save z, delta, and w vectors.

  // Update the execution trajectories.
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateC3ExecutionTrajectory(x_lcs_curr, t);
  UpdateRepositioningExecutionTrajectory(x_lcs_curr, t);

  record_timing("Execution trajectory updates");

  if (verbose_) {
    std::cout << "x_pred_curr_plan_ after updating: "
              << x_pred_curr_plan_.transpose() << std::endl;
    vector<VectorXd> zs = c3_curr_plan_->GetFullSolution();
    for (int i = 0; i < N_; i++) {
      std::cout << "z[" << i << "]: " << zs[i].transpose() << std::endl;
    }
    LCS verbose_lcs = lcs_candidates.at(SampleIndex::kCurrentLocation);
    MatrixXd E = verbose_lcs.E()[0];
    MatrixXd F = verbose_lcs.F()[0];
    MatrixXd H = verbose_lcs.H()[0];
    VectorXd c = verbose_lcs.c()[0];
    std::cout << "\nRight side of complementarity: " << std::endl;
    for (int i = 0; i < N_; i++) {
      VectorXd x = zs[i].head(n_x_);
      VectorXd lambda = zs[i].segment(n_x_, n_lambda_);
      VectorXd u = zs[i].tail(n_u_);
      std::cout << "\t" << i << ": "
                << (E * x + F * lambda + H * u + c).transpose() << std::endl;
    }
    std::cout << "\nComplementarity violation: " << std::endl;
    for (int i = 0; i < N_; i++) {
      VectorXd x = zs[i].head(n_x_);
      VectorXd lambda = zs[i].segment(n_x_, n_lambda_);
      VectorXd u = zs[i].tail(n_u_);
      std::cout << "\t" << i << ": "
                << lambda.dot(E * x + F * lambda + H * u + c) << std::endl;
    }

    std::cout << "Dynamically feasible ee current plan: " << std::endl;
    for (int i = 0; i < N_ + 1; i++) {
      std::cout << all_sample_dynamically_feasible_plans_
                       .at(SampleIndex::kCurrentLocation)[i]
                       .segment(0, 3)
                       .transpose()
                << std::endl;
    }

    std::cout << "Dynamically feasible object current plan: " << std::endl;
    for (int i = 0; i < N_ + 1; i++) {
      std::cout << all_sample_dynamically_feasible_plans_
                       .at(SampleIndex::kCurrentLocation)[i]
                       .segment(n_q_ - 7, 7)
                       .transpose()
                << std::endl;
    }
  }

  record_timing("Verbose diagnostics");

  // Add delay.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(controller_params_.control_loop_delay_ms));

  record_timing("Injected control-loop delay");

  // End of control loop cleanup.
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  filtered_solve_time_ = (1 - solve_time_filter_constant_) * solve_time +
                         (solve_time_filter_constant_)*filtered_solve_time_;

  record_timing("End-of-loop cleanup");

  std::cout << "ComputePlan timing breakdown [ms]:" << std::endl;
  for (const auto& timing_entry : timing_breakdown_ms) {
    std::cout << "  " << timing_entry.first << ": " << timing_entry.second
              << std::endl;
  }
  std::cout << "  Overall ComputePlan: " << solve_time * 1e3 << std::endl;

  if (verbose_) {
    std::cout << "At end of loop solve_time: " << solve_time << std::endl;
    std::cout << "At end of loop filtered_solve_time_: " << filtered_solve_time_
              << std::endl;
  }
  return drake::systems::EventStatus::Succeeded();
}

// Use a predicted EE state, if opted by controller settings.  The predicted
// location is clamped to a reasonable distance from the current EE location.
// Optionally, there is a reset mechanism to prevent using a predicted EE
// location if the last state is closer to the current state than the
// prediction.
void SamplingC3Controller::ResolvePredictedEEState(
    const bool& is_teleop, drake::VectorX<double>& x_lcs_curr) const {
  // Store the current actual state before applying prediction in preparation
  // for next control loop.
  x_from_last_control_loop_ = x_lcs_curr;

  // Detect if prediction is requested in current mode.
  bool in_c3_mode_and_predict =
      sampling_c3_options_.use_predicted_x0_c3 && is_doing_c3_;
  bool in_repos_mode_and_predict =
      sampling_c3_options_.use_predicted_x0_repos && !is_doing_c3_;

  if (!x_pred_curr_plan_.isZero() && !is_teleop &&
      (in_c3_mode_and_predict || in_repos_mode_and_predict)) {
    // Consider the current, last, and predicted states.
    Vector3d curr_ee = x_lcs_curr.head(3);
    Vector3d last_ee = x_from_last_control_loop_.head(3);
    Vector3d pred_ee = x_pred_from_last_control_loop_.head(3);

    if (((curr_ee - last_ee).norm() < (curr_ee - pred_ee).norm()) &&
        (curr_ee - pred_ee).norm() > 0.01 &&
        !x_pred_from_last_control_loop_.isZero() &&
        sampling_c3_options_.use_predicted_x0_reset_mechanism) {
      // Skip using the predicted state.
      if (verbose_) {
        std::cout << "RESET x_pred in mode: C3 " << in_c3_mode_and_predict
                  << ", or Repositioning " << in_repos_mode_and_predict
                  << std::endl;
        std::cout << "curr_ee-last_ee is " << (curr_ee - last_ee).norm()
                  << " and curr_ee-pred_ee is " << (curr_ee - pred_ee).norm()
                  << std::endl;
        std::cout << "x_lcs_curr without clamping: " << x_lcs_curr.transpose()
                  << std::endl;
      }
    } else {
      // Do the clamping.
      ClampEndEffectorAcceleration(x_lcs_curr);
      if (verbose_) {
        std::cout << "x_lcs_curr after clamping in mode: C3 "
                  << in_c3_mode_and_predict << ", or Repositioning "
                  << in_repos_mode_and_predict << " --> "
                  << x_lcs_curr.transpose() << std::endl;
      }
    }
  }

  // Store the predicted actual state in preparation for next control loop.
  x_pred_from_last_control_loop_ = x_lcs_curr;
}

// Clamp end effector acceleration if using predicted state.
void SamplingC3Controller::ClampEndEffectorAcceleration(
    drake::VectorX<double>& x_lcs_curr) const {
  // Use fixed approximate loop time for acceleration capping heuristic.
  float approx_loop_dt = std::min(0.1, filtered_solve_time_);
  float nominal_accel = sampling_c3_options_.nominal_ee_accel;
  for (int i = 0; i < 3; i++) {
    x_lcs_curr[i] = std::clamp(
        x_pred_curr_plan_[i],
        x_lcs_curr[i] - nominal_accel * approx_loop_dt * approx_loop_dt,
        x_lcs_curr[i] + nominal_accel * approx_loop_dt * approx_loop_dt);
    x_lcs_curr[n_q_ + i] =
        std::clamp(x_pred_curr_plan_[n_q_ + i],
                   x_lcs_curr[n_q_ + i] - nominal_accel * approx_loop_dt,
                   x_lcs_curr[n_q_ + i] + nominal_accel * approx_loop_dt);
  }
}

// Check for workspace limit violations.  If violated, the controller errors and
// stops.
void SamplingC3Controller::CheckForWorkspaceLimitViolations(
    const TimestampedVector<double>* lcs_x_curr) const {
  // xyz checks
  for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
    DRAKE_DEMAND(lcs_x_curr->get_data().segment(0, 3).transpose() *
                     sampling_c3_options_.workspace_limits[i].segment(0, 3) >
                 sampling_c3_options_.workspace_limits[i][3]);
    DRAKE_DEMAND(lcs_x_curr->get_data().segment(0, 3).transpose() *
                     sampling_c3_options_.workspace_limits[i].segment(0, 3) <
                 sampling_c3_options_.workspace_limits[i][4]);
  }
  // radius checks
  DRAKE_DEMAND(std::pow(lcs_x_curr->get_data()[0], 2) +
                   std::pow(lcs_x_curr->get_data()[1], 2) >
               std::pow(sampling_c3_options_.robot_radius_limits[0], 2));
  DRAKE_DEMAND(std::pow(lcs_x_curr->get_data()[0], 2) +
                   std::pow(lcs_x_curr->get_data()[1], 2) <
               std::pow(sampling_c3_options_.robot_radius_limits[1], 2));
}

// Update the cost matrices (Q_, R_, G_, U_) in preparation for C3 solves.
// Handle quaternion-dependent cost if enabled.
void SamplingC3Controller::UpdateCostMatrices(
    const drake::VectorX<double>& x_lcs_curr,
    const BasicVector<double>& x_lcs_des, const C3Options& c3_options) const {
  Q_.clear();
  R_.clear();
  G_.clear();
  U_.clear();
  double discount_factor = 1;

  for (int i = 0; i < N_ + 1; ++i) {
    Q_.push_back(discount_factor * c3_options.Q);
    discount_factor *= c3_options.gamma;
    if (i < N_) {
      R_.push_back(discount_factor * c3_options.R);
      G_.push_back(c3_options.G);
      U_.push_back(c3_options.U);
    }
  }

  if (sampling_c3_options_.use_quaternion_dependent_cost &&
      crossed_cost_switching_threshold_) {
    for (int i = 0; i < controller_params_.num_objects; i++) {
      VectorXd quat = (x_lcs_curr.segment(3 + 7 * i, 4));
      VectorXd quat_desired = x_lcs_des.get_value().segment(3 + 7 * i, 4);
      MatrixXd Q_quaternion_dependent_cost =
          hessian_of_squared_quaternion_angle_difference(quat, quat_desired);
      // Get the eigenvalues of the hessian to regularize so the Q matrix is
      // always PSD.
      double min_eigval =
          Q_quaternion_dependent_cost.eigenvalues().real().minCoeff();
      MatrixXd Q_quaternion_dependent_regularizers_part_1 =
          std::max(0.0, -min_eigval) * MatrixXd::Identity(4, 4);
      MatrixXd Q_quaternion_dependent_regularizers_part_2 =
          quat_desired * quat_desired.transpose();
      DRAKE_ASSERT(Q_quaternion_dependent_cost.rows() == 4 &&
                   Q_quaternion_dependent_cost.cols() == 4 &&
                   Q_quaternion_dependent_regularizers_part_1.rows() == 4 &&
                   Q_quaternion_dependent_regularizers_part_1.cols() == 4 &&
                   Q_quaternion_dependent_regularizers_part_2.rows() == 4 &&
                   Q_quaternion_dependent_regularizers_part_2.cols() == 4);
      double discount_factor = 1;
      for (int timestep_i = 0; timestep_i < N_ + 1; ++timestep_i) {
        Q_[timestep_i].block(3 + 7 * i, 3 + 7 * i, 4, 4) =
            discount_factor *
            sampling_c3_options_.q_quaternion_dependent_weight *
            (Q_quaternion_dependent_cost +
             Q_quaternion_dependent_regularizers_part_1 +
             sampling_c3_options_.q_quaternion_dependent_regularizer_fraction *
                 Q_quaternion_dependent_regularizers_part_2);
        discount_factor *= sampling_c3_options_.gamma;
      }
    }
  }

  if (verbose_) {
    std::cout << "Q_[0] with gamma " << sampling_c3_options_.gamma << ":"
              << std::endl;
    std::cout << Q_[0] << std::endl;
    std::cout << "R_[0] with gamma " << sampling_c3_options_.gamma << ":"
              << std::endl;
    std::cout << R_[0] << std::endl;
  }
}

vector<SortedPair<GeometryId>> SamplingC3Controller::GetResolvedContactPairs(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms,
    const vector<int>& resolve_contacts_to_list,
    vector<int> num_friction_directions, bool verbose) const {
  int n_contacts = std::accumulate(resolve_contacts_to_list.begin(),
                                   resolve_contacts_to_list.end(), 0);
  vector<SortedPair<GeometryId>> resolved_contacts;
  resolved_contacts.clear();

  // contact_geoms represents contact pair groups, where each group may contain
  // contact pairs between two objects, or between one object and multiple
  // objects eg. the end-effector and multiple objects in the environment.
  // resolve_contacts_to_list represents how many contact pairs to resolve to
  // for each contact pair group. For each contact pair group, select the
  // closest contact pairs up to the number specified by
  // resolve_contacts_to_list for that group, and add those to the
  // resolved_contacts list.
  for (int i = 0; i < contact_geoms.size(); i++) {
    DRAKE_DEMAND(contact_geoms[i].size() >= resolve_contacts_to_list[i]);

    const auto& candidates = contact_geoms[i];
    const int num_to_select = resolve_contacts_to_list[i];

    auto active_contacts = LCSFactory::GetNClosestContactPairs(
        plant, context, contact_geoms[i], num_to_select);
    if (!active_contacts.empty()) {
      resolved_contacts.insert(resolved_contacts.end(), active_contacts.begin(),
                               active_contacts.end());
    }
  }
  DRAKE_DEMAND(resolved_contacts.size() == n_contacts);
  return resolved_contacts;
}

// Create LCS objects (for the C3 solve and also for the C3 cost calculation)
// for each sample.
std::pair<vector<LCS>, vector<LCS>>
SamplingC3Controller::CreateLCSObjectsForSamples(
    const vector<VectorXd>& candidate_states,
    const drake::VectorX<double>& x_lcs_curr,
    const LCSFactoryOptions& lcs_factory_options) const {
  vector<LCS> lcs_candidates;
  vector<LCS> lcs_candidates_for_cost;

  int num_total_samples = candidate_states.size();
  for (int i = 0; i < num_total_samples; i++) {
    // Context needs to be updated to create the LCS objects.
    UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                  candidate_states[i]);

    // Resolve the contact pairs and create the LCS.
    vector<SortedPair<GeometryId>> resolved_contact_pairs =
        GetResolvedContactPairs(
            plant_, *context_, contact_pairs_,
            sampling_c3_options_.resolve_contacts_to,
            sampling_c3_options_.num_friction_directions_per_contact.value(),
            verbose_);
    LCS lcs_object_sample =
        LCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                   resolved_contact_pairs, lcs_factory_options)
            .GenerateLCS();
    lcs_candidates.push_back(lcs_object_sample);

    // Create different LCS objects for cost calculation.
    vector<SortedPair<GeometryId>> resolved_contact_pairs_for_cost_simulation;
    resolved_contact_pairs_for_cost_simulation = GetResolvedContactPairs(
        plant_, *context_, contact_pairs_,
        sampling_c3_options_.resolve_contacts_to_for_cost,
        sampling_c3_options_.num_friction_directions_per_contact_for_cost,
        verbose_);
    LCSFactoryOptions lcs_factory_options_for_cost = {
        .contact_model = controller_params_.sampling_c3_options.contact_model,
        .N = N_ * sampling_c3_options_.lcs_dt_resolution,
        .dt = dt_ / sampling_c3_options_.lcs_dt_resolution,
        .num_contacts = resolved_contact_pairs_for_cost_simulation.size(),
        .spring_stiffness = 0.0,
        .num_friction_directions_per_contact =
            sampling_c3_options_.num_friction_directions_per_contact_for_cost,
        .mu_per_contact = sampling_c3_options_.mu_for_cost,
        .planar_normal_direction =
            sampling_c3_options_.planar_normal_direction};
    LCS lcs_object_sample_for_cost_simulation =
        LCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                   resolved_contact_pairs_for_cost_simulation,
                   lcs_factory_options_for_cost)
            .GenerateLCS();
    lcs_candidates_for_cost.push_back(lcs_object_sample_for_cost_simulation);
  }

  // Reset the context to the current lcs state.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);

  if (verbose_) {
    // Print the LCS matrices for verbose inspection.
    LCS verbose_lcs = lcs_candidates.at(SampleIndex::kCurrentLocation);
    std::cout << "A: " << std::endl;
    std::cout << verbose_lcs.A()[0] << std::endl;
    std::cout << "B: " << std::endl;
    std::cout << verbose_lcs.B()[0] << std::endl;
    std::cout << "D: " << std::endl;
    std::cout << verbose_lcs.D()[0] << std::endl;
    std::cout << "d: " << std::endl;
    std::cout << verbose_lcs.d()[0] << std::endl;
    std::cout << "E: " << std::endl;
    std::cout << verbose_lcs.E()[0] << std::endl;
    std::cout << "F: " << std::endl;
    std::cout << verbose_lcs.F()[0] << std::endl;
    std::cout << "H: " << std::endl;
    std::cout << verbose_lcs.H()[0] << std::endl;
    std::cout << "c: " << std::endl;
    std::cout << verbose_lcs.c()[0] << std::endl;
  }

  return std::make_pair(lcs_candidates, lcs_candidates_for_cost);
}

void SamplingC3Controller::UpdateC3ExecutionTrajectory(
    const VectorXd& x_lcs, const double& t_context) const {
  // Get the input and full state solution from the plan.
  vector<VectorXd> u_sol = c3_curr_plan_->GetInputSolution();
  vector<VectorXd> x_sol = c3_curr_plan_->GetStateSolution();

  // TODO @bibit:  this is hacky and affects non-planar examples
  // if (x_sol[0][2] >= 0.03) {
  //   for (int i = 0; i < x_sol.size(); ++i) {
  //     x_sol[i][2] -= 0.01;
  //   }
  // }

  // Setting up matrices to set up LCMTrajectory object.
  MatrixXd knots = MatrixXd::Zero(n_x_, N_);
  VectorXd timestamps = VectorXd::Zero(N_);

  // Set up matrices for LCMTrajectory object.
  for (int i = 0; i < N_; i++) {
    knots.col(i) = x_sol[i];
    timestamps[i] = t_context + filtered_solve_time_ + (i)*dt_;
  }

  // Update predicted next state if in this mode.
  if (is_doing_c3_) {
    if (filtered_solve_time_ < (N_ - 1) * dt_) {
      int last_passed_index = filtered_solve_time_ / dt_;
      double fraction_to_next_index =
          (filtered_solve_time_ / dt_) - (double)last_passed_index;
      x_pred_curr_plan_ =
          knots.col(last_passed_index) +
          fraction_to_next_index *
              (knots.col(last_passed_index + 1) - knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  double wall_offset = 0;

  if (sampling_c3_options_.include_walls && sampling_params_.sample_on_wall) {
    double x_min = sampling_c3_options_.workspace_limits[0][3];
    double x_max = sampling_c3_options_.workspace_limits[0][4];
    double y_min = sampling_c3_options_.workspace_limits[1][3];
    double y_max = sampling_c3_options_.workspace_limits[1][4];

    // if ee is close to wall, raise z_height
    if ((x_sol[0][0] <= x_min + 0.05 &&
         x_sol[0][0] >= x_min - sampling_c3_options_.workspace_margins) ||
        (x_sol[0][0] >= x_max - 0.05 &&
         x_sol[0][0] <= x_max + sampling_c3_options_.workspace_margins) ||
        (x_sol[0][1] <= y_min + 0.05 &&
         x_sol[0][1] >= y_min - sampling_c3_options_.workspace_margins) ||
        (x_sol[0][1] >= y_max - 0.05 &&
         x_sol[0][1] <= y_max + sampling_c3_options_.workspace_margins)) {
      wall_offset = 0.01;
    }
  }

  // TODO @bibit:  this breaks non-planar examples
  // for (int i = 0; i < N_; i++) {
  //   knots(2, i) =
  //       sampling_params_.z_height + wall_offset;  // keep ee height constant
  //   knots(n_q_ + 2, i) = 0;                       // keep ee z-velo constant
  // }

  // Add end effector position target to LCM Trajectory.
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = vector<std::string>(3, "double");
  ee_traj.datapoints = knots(seqN(0, 3), seqN(0, N_));
  ee_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.ClearTrajectories();
  c3_execution_lcm_traj_.AddTrajectory(ee_traj.traj_name, ee_traj);

  // Add end effector orientation target, if enabled.
  IncludeEEOrientationTargetIfEnabled(&c3_execution_lcm_traj_,
                                      ee_position_curr_, timestamps);

  // Add end effector force target to LCM Trajectory.
  // In c3 mode, the end effector forces should match the solved c3 inputs.
  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  for (int i = 0; i < N_; i++) {
    force_samples.col(i) = u_sol[i];
  }
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);

  // No need to add object position and orientation since these are outputs sent
  // in the C3 plans.
}

// Compute repositioning trajectory.
void SamplingC3Controller::UpdateRepositioningExecutionTrajectory(
    const VectorXd& x_lcs, const double& t_context) const {
  // Get the best sample location.
  Vector3d best_sample_location = all_sample_locations_[best_sample_index_];
  // Update the previous repositioning target for reference in next loop.
  prev_repositioning_target_ = best_sample_location;

  // Generate knot points according to the repositioning strategy.
  MatrixXd knots = Reposition(n_q_, n_x_, N_, x_lcs, best_sample_location, dt_,
                              is_doing_c3_, finished_reposition_flag_,
                              reposition_params_, sampling_c3_options_);

  // Update predicted next state if in this mode.
  if (!is_doing_c3_) {
    if (filtered_solve_time_ < (N_ - 1) * dt_) {
      int last_passed_index = filtered_solve_time_ / dt_;
      double fraction_to_next_index =
          (filtered_solve_time_ / dt_) - (double)last_passed_index;
      x_pred_curr_plan_ =
          knots.col(last_passed_index) +
          fraction_to_next_index *
              (knots.col(last_passed_index + 1) - knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  // Set up the trajectory.
  VectorXd timestamps = VectorXd::Zero(N_);
  for (int i = 0; i < N_; i++) {
    timestamps[i] = t_context + filtered_solve_time_ + (i)*dt_;
  }

  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = vector<std::string>(3, "double");
  ee_traj.datapoints = knots(seqN(0, 3), seqN(0, N_));
  ee_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.ClearTrajectories();
  repos_execution_lcm_traj_.AddTrajectory(ee_traj.traj_name, ee_traj);

  // Include end effector orientation targets, if enabled.
  IncludeEEOrientationTargetIfEnabled(&repos_execution_lcm_traj_,
                                      ee_position_curr_, timestamps);

  // In repositioning mode, the end effector should not exert forces.
  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);

  // No need to add object position and orientation.
}

// Prune outdated samples from a sample buffer, based on object motion.
void SamplingC3Controller::PruneOutdatedSamplesFromBuffer(
    const VectorXd& x_lcs, int* num_in_buffer, MatrixXd* sample_buffer,
    VectorXd* sample_costs_buffer, const double& pos_error_sample_retention,
    const double& ang_error_sample_retention) const {
  int n_buffer_length = sample_costs_buffer->size();
  // Get object positions and orientations, both current and from the buffer.
  vector<Array<bool, Dynamic, 1>> mask_satisfies_rot;
  vector<Array<bool, Dynamic, 1>> mask_satisfies_pos;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    Vector3d object_pos = x_lcs.segment(7 + 7 * i, 3);
    Vector4d object_quat = x_lcs.segment(3 + 7 * i, 4).normalized();
    MatrixXd buffer_xyzs =
        sample_buffer->block(0, 7 + 7 * i, n_buffer_length, 3);
    MatrixXd buffer_quats =
        sample_buffer->block(0, 3 + 7 * i, n_buffer_length, 4);

    // Compute the angular difference.
    VectorXd quat_dots = (buffer_quats * object_quat).array().abs();
    quat_dots = quat_dots.cwiseMax(-1.0).cwiseMin(1.0);
    VectorXd angles = 2.0 * quat_dots.array().acos();
    mask_satisfies_rot.push_back(angles.array() < ang_error_sample_retention);

    // Compute the linear difference.
    MatrixXd pos_deltas = buffer_xyzs.rowwise() - object_pos.transpose();
    VectorXd distances = pos_deltas.rowwise().norm();
    mask_satisfies_pos.push_back(distances.array() <
                                 pos_error_sample_retention);
  }

  // Keep buffer if no objects moved.
  int retained_count = 0;
  MatrixXd retained_samples = MatrixXd::Zero(n_buffer_length, n_q_);
  VectorXd retained_costs = -1 * VectorXd::Ones(n_buffer_length);
  for (int i = 0; i < *num_in_buffer; i++) {
    if ((*sample_costs_buffer)[i] < 0) {
      break;
    }
    bool keep = true;
    for (int j = 0; j < controller_params_.num_objects; j++) {
      if (!(mask_satisfies_rot.at(j)[i] && mask_satisfies_pos.at(j)[i])) {
        keep = false;
        break;
      }
    }
    if (keep) {
      retained_samples.row(retained_count) = sample_buffer->row(i);
      retained_costs[retained_count] = (*sample_costs_buffer)[i];
      retained_count++;
    }
  }
  *num_in_buffer = retained_count;
  *sample_buffer = retained_samples;
  *sample_costs_buffer = retained_costs;
}

// Maintain the sample buffers (both for keeping track of unattempted samples
// and their costs, and of attempted unsuccessful samples):  prune outdated
// samples and add new.
void SamplingC3Controller::MaintainSampleBuffers(const VectorXd& x_lcs) const {
  // First, handle the unsuccessful sample buffer.  This buffer just needs to
  // prune outdated samples; new samples get added one at a time when the
  // controller goes from repositioning to C3 mode.
  PruneOutdatedSamplesFromBuffer(
      x_lcs, &num_in_unsuccessful_buffer_, &unsuccessful_sample_buffer_,
      &unsuccessful_sample_costs_buffer_,
      sampling_params_.unsuccessful_pos_error_sample_retention,
      sampling_params_.unsuccessful_ang_error_sample_retention);

  // Second, handle the unattempted sample buffer.  First, prune outdated
  // samples.
  PruneOutdatedSamplesFromBuffer(x_lcs, &num_in_buffer_, &sample_buffer_,
                                 &sample_costs_buffer_,
                                 sampling_params_.pos_error_sample_retention,
                                 sampling_params_.ang_error_sample_retention);
  int retained_count = num_in_buffer_;

  // Third, in preparation for adding new samples stored in
  // all_sample_locations_ (excluding the current location), if the buffer is
  // going to overflow, get rid of the oldest samples first.  NOTE:  Step 4
  // moves the lowest cost sample in the buffer to the end, so the best sample
  // is usually excluded from this cut.
  int num_to_add = all_sample_locations_.size() - 1;
  if (!is_doing_c3_ && all_sample_locations_.size() ==
                           sampling_params_.num_additional_samples_repos + 2) {
    // Don't add the repositioning target since it was a past sample and should
    // already be in the buffer.  The size check determines if the previous
    // repositioning target was rejected due to collision (in which case sample
    // index 1 is a new sample and should be added).
    num_to_add--;
  }
  if (retained_count + num_to_add > sampling_params_.N_sample_buffer) {
    int shift_by =
        retained_count + num_to_add - sampling_params_.N_sample_buffer;
    retained_count -= shift_by;
    sample_buffer_.block(0, 0, retained_count, n_q_) =
        sample_buffer_.block(shift_by, 0, retained_count, n_q_);
    sample_costs_buffer_.segment(0, retained_count) =
        sample_costs_buffer_.segment(shift_by, retained_count);
  }

  // Fourth, add the new samples stored in all_sample_locations_ and
  // all_sample_costs_.  Don't add the current location (so the sample buffer
  // contains more broadly sampled locations) or a currently pursued
  // repositioning target.  Remove the travel cost from the costs before adding
  // to the buffer.
  int buffer_count = retained_count;
  for (int i = 0; i < all_sample_locations_.size(); i++) {
    if ((i == 0) || (!is_doing_c3_ && i == 1 &&
                     all_sample_locations_.size() ==
                         sampling_params_.num_additional_samples_repos + 2)) {
      // Skip the current location.
      // Skip the repositioning target if in repositioning mode and if it was
      // not rejected due to collision.
    } else {
      // First ensure there is no attempt to write beyond the end of the buffer.
      DRAKE_DEMAND(buffer_count < sampling_params_.N_sample_buffer);

      // Add the new sample to the buffer.
      VectorXd new_config = x_lcs.head(n_q_);
      new_config.head(3) = all_sample_locations_[i];
      double travel_cost = progress_params_.travel_cost_per_meter *
                           (new_config.head(2) - x_lcs.head(2)).norm();
      // Ensure a normalized quaternion is written to the buffer.
      for (int j = 0; j < controller_params_.num_objects; j++) {
        Vector4d object_quat = x_lcs.segment(3 + 7 * j, 4).normalized();
        new_config.segment(3 + 7 * j, 4) = object_quat;
      }
      sample_buffer_.row(buffer_count) = new_config;
      sample_costs_buffer_[buffer_count] = all_sample_costs_[i] - travel_cost;
      buffer_count++;
    }
  }
  num_in_buffer_ = buffer_count;

  // Lastly, ensure the lowest cost sample is at the end of the buffer.  This
  // cost factors in the travel cost.
  VectorXd eligible_costs = sample_costs_buffer_.head(num_in_buffer_);
  // Incorporate travel costs for each sample in the buffer.
  MatrixXd xy_samples = sample_buffer_.block(0, 0, num_in_buffer_, 2);
  Vector2d xy_ref = x_lcs.head(2);
  VectorXd travel_costs =
      progress_params_.travel_cost_per_meter *
      (xy_samples.rowwise() - xy_ref.transpose()).rowwise().norm();
  eligible_costs += travel_costs;

  int lowest_cost_index;
  double lowest_buffer_cost = eligible_costs.minCoeff(&lowest_cost_index);
  VectorXd lowest_cost_sample = sample_buffer_.row(lowest_cost_index);
  sample_buffer_.row(lowest_cost_index) =
      sample_buffer_.row(num_in_buffer_ - 1);
  sample_costs_buffer_[lowest_cost_index] =
      sample_costs_buffer_[num_in_buffer_ - 1];
  sample_buffer_.row(num_in_buffer_ - 1) = lowest_cost_sample;
  sample_costs_buffer_[num_in_buffer_ - 1] = lowest_buffer_cost;

  DRAKE_DEMAND(sample_buffer_.rows() == sampling_params_.N_sample_buffer);
  DRAKE_DEMAND(sample_buffer_.cols() == n_q_);
  DRAKE_DEMAND(sample_costs_buffer_.size() == sampling_params_.N_sample_buffer);
}

// If eligible, augment the current control loop's considered samples with the
// best one from the buffer.
void SamplingC3Controller::AugmentSamplesWithBuffer(
    vector<std::shared_ptr<C3>>& c3_objects) const {
  // Add the best from the buffer to the samples, but only if in C3 mode and
  // only if the best in the buffer is distinct from the current set of samples.
  if ((is_doing_c3_) &&
      (sampling_params_.consider_best_buffer_sample_when_leaving_c3)) {
    // Get the lowest cost sample from the buffer, and incorporate travel cost.
    double lowest_buffer_cost = sample_costs_buffer_[num_in_buffer_ - 1];
    Vector2d best_buffer_xy = sample_buffer_.row(num_in_buffer_ - 1).head(2);
    double travel_cost =
        progress_params_.travel_cost_per_meter *
        (best_buffer_xy - all_sample_locations_[0].head(2)).norm();
    lowest_buffer_cost += travel_cost;
    Vector3d best_buffer_ee_sample =
        sample_buffer_.row(num_in_buffer_ - 1).head(3);

    // Get the lowest cost from the current set of samples (these already
    // incoporate travel cost).
    double lowest_new_cost =
        *std::min_element(all_sample_costs_.begin(), all_sample_costs_.end());
    vector<double>::iterator it = std::min_element(
        std::begin(all_sample_costs_), std::end(all_sample_costs_));
    int lowest_new_cost_index =
        (SampleIndex)(std::distance(std::begin(all_sample_costs_), it));
    Vector3d best_new_ee_sample = all_sample_locations_[lowest_new_cost_index];

    // Initialize the buffer plan with something.
    if (dynamically_feasible_buffer_plan_.size() != N_ + 1) {
      c3_buffer_plan_ = c3_objects[lowest_new_cost_index];
      dynamically_feasible_buffer_plan_ =
          all_sample_dynamically_feasible_plans_[lowest_new_cost_index];
    }
    // If the best in the buffer is from the current set of samples, store the
    // associated C3 object and dynamically feasible plan, but don't add to the
    // set of samples to consider for repositioning.
    else if ((abs(lowest_buffer_cost - lowest_new_cost) < 1e-5) &&
             ((best_buffer_ee_sample - best_new_ee_sample).norm() < 1e-5)) {
      c3_buffer_plan_ = c3_objects[lowest_new_cost_index];
      dynamically_feasible_buffer_plan_ =
          all_sample_dynamically_feasible_plans_[lowest_new_cost_index];
    }
    // If the best in the buffer is distinct from the current set of samples,
    // consider it for repositioning by adding it to the set of samples, costs,
    // etc.
    else {
      all_sample_costs_.push_back(lowest_buffer_cost);
      all_sample_locations_.push_back(best_buffer_ee_sample);
      c3_objects.push_back(c3_buffer_plan_);
      all_sample_dynamically_feasible_plans_.push_back(
          dynamically_feasible_buffer_plan_);
    }
  }
}

// Add the current state to the unsuccessful buffer.
void SamplingC3Controller::AddToUnsuccessfulBuffer(
    const VectorXd& x_lcs) const {
  // Check if the unsuccessful buffer is going to overflow.
  if (num_in_unsuccessful_buffer_ ==
      sampling_params_.N_unsuccessful_sample_buffer) {
    std::cout << "!!! WARNING !!! Unsuccessful sample buffer overflow"
              << std::endl;
    for (int i = 0; i < num_in_unsuccessful_buffer_ - 1; i++) {
      unsuccessful_sample_buffer_.row(i) =
          unsuccessful_sample_buffer_.row(i + 1);
      unsuccessful_sample_costs_buffer_[i] =
          unsuccessful_sample_costs_buffer_[i + 1];
    }
    num_in_unsuccessful_buffer_--;
  }
  // Add the current location to the unsuccessful buffer.
  unsuccessful_sample_buffer_.row(num_in_unsuccessful_buffer_) =
      x_lcs.head(n_q_);
  unsuccessful_sample_costs_buffer_[num_in_unsuccessful_buffer_] =
      all_sample_costs_[0];
  num_in_unsuccessful_buffer_++;

  // If desired, remove nearby samples from the unattempted sample buffer.
  if (sampling_params_.avoid_choosing_unsuccessful_samples) {
    int retained_count = 0;
    MatrixXd retained_samples =
        MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
    VectorXd retained_costs =
        -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);
    for (int i = 0; i < num_in_buffer_; i++) {
      // Check the EE position of the sample, and remove the sample from the
      // buffer if too close to the new unsuccessful sample.
      Vector3d ee_sample = sample_buffer_.row(i).head(3);
      double ee_dist = (ee_sample - x_lcs.head(3)).norm();
      if (ee_dist > sampling_params_.unsuccessful_radius) {
        retained_samples.row(retained_count) = sample_buffer_.row(i);
        retained_costs[retained_count] = sample_costs_buffer_[i];
        retained_count++;
      }
    }
    num_in_buffer_ = retained_count;
    sample_buffer_ = retained_samples;
    sample_costs_buffer_ = retained_costs;
  }
}

// Keep track of the progress made in the current C3 mode.
void SamplingC3Controller::KeepTrackOfC3ModeProgress(
    const drake::VectorX<double>& x_lcs_curr,
    const BasicVector<double>& x_lcs_final_des, bool& met_minimum_progress,
    const bool& print_current_pos_and_rot_cost) const {
  bool updated_cost = false;
  bool updated_config_cost = false;
  bool updated_pos_or_rot = false;
  double cost_progress_fraction = -INFINITY;  // Negative means progress.

  // Accumulate costs across all objects.
  double curr_pos_and_rot_cost = 0;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd Q_pos_and_rot = Q_[0].block(3 + 7 * i, 3 + 7 * i, 7, 7);
    VectorXd pos_and_rot_error_vec =
        x_lcs_curr.segment(3 + 7 * i, 7) -
        x_lcs_final_des.get_value().segment(3 + 7 * i, 7);
    curr_pos_and_rot_cost += pos_and_rot_error_vec.transpose() * Q_pos_and_rot *
                             pos_and_rot_error_vec;
  }

  // Check for progress along different metrics.
  if ((all_sample_costs_[SampleIndex::kCurrentLocation] < lowest_cost_) ||
      (lowest_cost_ == -1.0)) {
    lowest_cost_ = all_sample_costs_[SampleIndex::kCurrentLocation];
    updated_cost = true;
  }
  if ((curr_pos_and_rot_cost < lowest_pos_and_rot_current_cost_) ||
      (lowest_pos_and_rot_current_cost_ == -1.0)) {
    lowest_pos_and_rot_current_cost_ = curr_pos_and_rot_cost;
    updated_config_cost = true;
  }
  if ((current_position_error_ < lowest_position_error_) ||
      (lowest_position_error_ == -1.0)) {
    lowest_position_error_ = current_position_error_;
    updated_pos_or_rot = true;
  }
  if ((current_orientation_error_ < lowest_orientation_error_) ||
      (lowest_orientation_error_ == -1.0)) {
    lowest_orientation_error_ = current_orientation_error_;
    updated_pos_or_rot = true;
  }

  // One of the progress metrics requires a history of object configuration
  // costs.  Maintain this history and check for progress.
  object_config_cost_history_.push(curr_pos_and_rot_cost);
  int max_history_length = progress_params_.progress_enforced_over_n_loops;
  if (object_config_cost_history_.size() > max_history_length) {
    object_config_cost_history_.pop();
  }
  // Check for progress if the history is full.
  if (object_config_cost_history_.size() == max_history_length) {
    // Note:  front() is the oldest cost, back() is the most recent.
    cost_progress_fraction =  // Negative means progress.
        ((object_config_cost_history_.back() -
          object_config_cost_history_.front()) /
         object_config_cost_history_.front());
  }

  if (print_current_pos_and_rot_cost) {
    std::cout << "Current rot and pos cost: " << curr_pos_and_rot_cost
              << std::endl;
  }

  // Keep track of how many control loops have passed since the best seen
  // progress metric in this mode.
  ProgressMetric progress_metric = progress_params_.track_c3_progress_via;
  if (((progress_metric == ProgressMetric::kC3Cost) && updated_cost) ||
      ((progress_metric == ProgressMetric::kConfigCost) &&
       updated_config_cost) ||
      ((progress_metric == ProgressMetric::kPosOrRotCost) &&
       updated_pos_or_rot)) {
    best_progress_steps_ago_ = 0;
  } else {
    best_progress_steps_ago_++;
  }

  // Detect if progress was sufficient according to progress metric.
  int num_control_loops_to_wait = progress_params_.num_control_loops_to_wait;
  if (!crossed_cost_switching_threshold_) {
    num_control_loops_to_wait =
        progress_params_.num_control_loops_to_wait_position;
  }
  if (progress_metric == ProgressMetric::kConfigCostDrop) {
    if (cost_progress_fraction >
        -progress_params_.progress_enforced_cost_drop) {
      met_minimum_progress = false;
    }
  } else if (best_progress_steps_ago_ > num_control_loops_to_wait) {
    met_minimum_progress = false;
  }
}

// Reset the metrics used to track progress in C3 mode.
void SamplingC3Controller::ResetProgressMetrics() const {
  lowest_cost_ = -1.0;
  lowest_pos_and_rot_current_cost_ = -1.0;
  lowest_position_error_ = -1.0;
  lowest_orientation_error_ = -1.0;
  best_progress_steps_ago_ = 0;
  // Clear the stored history of object configuration costs.
  while (!object_config_cost_history_.empty()) {
    object_config_cost_history_.pop();
  }
}

void SamplingC3Controller::ResetSampleBuffers() const {
  sample_buffer_ = MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
  sample_costs_buffer_ = -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);
  num_in_buffer_ = 0;
  unsuccessful_sample_buffer_ =
      MatrixXd::Zero(sampling_params_.N_unsuccessful_sample_buffer, n_q_);
  unsuccessful_sample_costs_buffer_ =
      -1 * VectorXd::Ones(sampling_params_.N_unsuccessful_sample_buffer);
  num_in_unsuccessful_buffer_ = 0;
}

void SamplingC3Controller::IncludeEEOrientationTargetIfEnabled(
    LcmTrajectory* lcm_trajectory, const Vector3d& ee_position,
    const VectorXd& timestamps) const {
  // Add end effector orientation target to the provided LCM trajectory, if
  // adaptive EE tilt is enabled.  The orientation depends on the EE position
  // provided relative to the world center.
  if (adaptive_ee_tilt_) {
    MatrixXd ee_orientations = MatrixXd::Zero(4, N_);

    Vector3d direction = ee_position - workspace_center_;
    Matrix3d rot;
    rot << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    direction[2] = 0;
    direction = rot * direction;

    // If outside of radius, tilt ee so away from workspace center, otherwise
    // set vertical Tilt depending on how far from center (for smoothness)
    double theta = (direction.norm() / max_ee_dist_from_workspace_center_) *
                   reposition_params_.max_tilt_angle * M_PI / 180.0;
    direction.normalize();

    AngleAxisd angle_axis(theta, direction);
    Quaterniond q_rotated(angle_axis);
    Vector4d q_vec(q_rotated.w(), q_rotated.x(), q_rotated.y(), q_rotated.z());

    for (int i = 0; i < N_; i++) {
      ee_orientations.col(i) = q_vec;
    }

    LcmTrajectory::Trajectory ee_orientation_traj;
    ee_orientation_traj.traj_name = "end_effector_orientation_target";
    ee_orientation_traj.datatypes =
        vector<std::string>(ee_orientations.rows(), "double");  // quaternion
    ee_orientation_traj.datapoints = ee_orientations;
    ee_orientation_traj.time_vector = timestamps.cast<double>();

    lcm_trajectory->AddTrajectory(ee_orientation_traj.traj_name,
                                  ee_orientation_traj);
  }
}

// Output port handlers for current location
void SamplingC3Controller::OutputC3SolutionCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  auto z_sol = c3_curr_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.bottomRows(n_v_).topRows(3).cast<double>();

  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes = vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // Include end effector orientation targets, if enabled.
  IncludeEEOrientationTargetIfEnabled(&lcm_traj, ee_position_curr_,
                                      c3_solution->time_vector_.cast<double>());

  MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);
  auto z_sol = c3_curr_plan_->GetFullSolution();

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  // Add object position targets.
  vector<LcmTrajectory::Trajectory> object_trajs;
  vector<std::string> traj_names;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd knots = MatrixXd::Zero(6, N_);
    knots.topRows(3) =
        c3_solution->x_sol_.middleRows(7 + 7 * i, 3).cast<double>();
    knots.bottomRows(3) =
        c3_solution->x_sol_.middleRows(n_q_ + 6 + 6 * i, 3).cast<double>();

    LcmTrajectory::Trajectory object_traj;
    object_traj.traj_name = "object_position_target_" + std::to_string(i);
    object_traj.datatypes = vector<std::string>(knots.rows(), "double");
    object_traj.datapoints = knots;
    object_traj.time_vector = c3_solution->time_vector_.cast<double>();

    object_trajs.push_back(object_traj);
    traj_names.push_back("object_position_target_" + std::to_string(i));
  }
  LcmTrajectory lcm_traj(object_trajs, traj_names, "object_targets",
                         "object_targets", false);

  // Add object orientation targets.
  LcmTrajectory::Trajectory object_orientation_traj;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd orientation_sample = MatrixXd::Zero(4, N_);
    orientation_sample =
        c3_solution->x_sol_.middleRows(3 + 7 * i, 4).cast<double>();

    object_orientation_traj.traj_name =
        "object_orientation_target_" + std::to_string(i);
    object_orientation_traj.datatypes =
        vector<std::string>(orientation_sample.rows(), "double");
    object_orientation_traj.datapoints = orientation_sample;
    object_orientation_traj.time_vector =
        c3_solution->time_vector_.cast<double>();
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           object_orientation_traj);
  }

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  auto z_sol = c3_curr_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
}

void SamplingC3Controller::OutputC3IntermediatesCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;
  auto z_sol_curr_plan = c3_curr_plan_->GetFullSolution();
  auto delta_curr_plan = c3_curr_plan_->GetDualDeltaSolution();
  auto w_curr_plan = c3_curr_plan_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * dt_;
    c3_intermediates->z_.col(i) = z_sol_curr_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_curr_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_curr_plan[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianCurrPlan(
    const drake::systems::Context<double>& context,
    vector<LCSContactDescription>* lcs_contact_descriptions) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                lcs_x->get_data());

  LCSFactoryOptions lcs_factory_options =
      sampling_c3_options_.GetLCSFactoryOptions(
          crossed_cost_switching_threshold_);

  // Preprocessing the contact pairs
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  resolved_contact_pairs = GetResolvedContactPairs(
      plant_, *context_, contact_pairs_,
      sampling_c3_options_.resolve_contacts_to,
      sampling_c3_options_.num_friction_directions_per_contact.value(),
      verbose_);

  // print size of resolved_contact_pairs
  *lcs_contact_descriptions =
      LCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                 resolved_contact_pairs, lcs_factory_options)
          .GetContactDescriptions();
}

// Output port handlers for best sample location
void SamplingC3Controller::OutputC3SolutionBestPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  auto z_sol = c3_best_plan_->GetFullSolution();
  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.bottomRows(n_v_).topRows(3).cast<double>();

  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes = vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // Include end effector orientation targets, if enabled.
  IncludeEEOrientationTargetIfEnabled(&lcm_traj, ee_position_curr_,
                                      c3_solution->time_vector_.cast<double>());

  // Include end effector force targets.
  MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionBestPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  auto z_sol = c3_best_plan_->GetFullSolution();
  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  // Add object position targets.
  vector<LcmTrajectory::Trajectory> object_trajs;
  vector<std::string> traj_names;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd knot = MatrixXd::Zero(6, N_);
    knot.topRows(3) =
        c3_solution->x_sol_.middleRows(7 * i + 7, 3).cast<double>();
    knot.bottomRows(3) =
        c3_solution->x_sol_.middleRows(n_q_ + 6 * i + 6, 3).cast<double>();
    LcmTrajectory::Trajectory object_traj;
    object_traj.traj_name = "object_position_target_" + std::to_string(i);
    object_traj.datatypes = vector<std::string>(knot.rows(), "double");
    object_traj.datapoints = knot;
    object_traj.time_vector = c3_solution->time_vector_.cast<double>();

    object_trajs.push_back(object_traj);
    traj_names.push_back("object_position_target_" + std::to_string(i));
  }

  LcmTrajectory lcm_traj(object_trajs, traj_names, "object_targets",
                         "object_targets", false);

  // Add object orientation targets.
  for (int i = 0; i < controller_params_.num_objects; i++) {
    LcmTrajectory::Trajectory object_orientation_traj;
    MatrixXd orientation_sample = MatrixXd::Zero(4, N_);
    orientation_sample =
        c3_solution->x_sol_.middleRows(3 + 7 * i, 4).cast<double>();

    object_orientation_traj.traj_name =
        "object_orientation_target_" + std::to_string(i);
    object_orientation_traj.datatypes =
        vector<std::string>(orientation_sample.rows(), "double");
    object_orientation_traj.datapoints = orientation_sample;
    object_orientation_traj.time_vector =
        c3_solution->time_vector_.cast<double>();
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           object_orientation_traj);
  }

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

// TODO @bibit:  continue looking for changes from here
void SamplingC3Controller::OutputC3SolutionBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;
  auto z_sol = c3_best_plan_->GetFullSolution();

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
}

void SamplingC3Controller::OutputC3IntermediatesBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;
  auto z_sol_best_plan = c3_best_plan_->GetFullSolution();
  auto delta_best_plan = c3_best_plan_->GetDualDeltaSolution();
  auto w_best_plan = c3_best_plan_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * dt_;
    c3_intermediates->z_.col(i) = z_sol_best_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_best_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_best_plan[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianBestPlan(
    const drake::systems::Context<double>& context,
    vector<LCSContactDescription>* lcs_contact_descriptions) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Linearize about state with end effector in sample location.
  VectorXd x_sample = lcs_x->get_data();
  x_sample.head(3) = all_sample_locations_[best_sample_index_];
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_sample);

  LCSFactoryOptions lcs_factory_options =
      sampling_c3_options_.GetLCSFactoryOptions(
          crossed_cost_switching_threshold_);

  // Preprocess the contact pairs.
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  resolved_contact_pairs = GetResolvedContactPairs(
      plant_, *context_, contact_pairs_,
      sampling_c3_options_.resolve_contacts_to,
      sampling_c3_options_.num_friction_directions_per_contact.value(),
      verbose_);
  *lcs_contact_descriptions =
      LCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                 resolved_contact_pairs, lcs_factory_options)
          .GetContactDescriptions();

  // Revert the context.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                lcs_x->get_data());
}

// Output port handlers for executing C3 and repositioning ports
void SamplingC3Controller::OutputC3TrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_c3_execution_lcm_traj) const {
  // Returned trajectory includes EE positions and feed-forward forces.
  LcmTrajectory::Trajectory end_effector_traj =
      c3_execution_lcm_traj_.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  if (adaptive_ee_tilt_) {
    LcmTrajectory::Trajectory ee_orientation_traj =
        c3_execution_lcm_traj_.GetTrajectory("end_effector_orientation_target");
    lcm_traj.AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);
  }

  LcmTrajectory::Trajectory force_traj =
      c3_execution_lcm_traj_.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_c3_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_c3_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputReposTrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_repos_execution_lcm_traj)
    const {
  // Returned trajectory includes EE positions and feed-forward forces.
  LcmTrajectory::Trajectory end_effector_traj =
      repos_execution_lcm_traj_.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  if (adaptive_ee_tilt_) {
    LcmTrajectory::Trajectory ee_orientation_traj =
        repos_execution_lcm_traj_.GetTrajectory(
            "end_effector_orientation_target");
    lcm_traj.AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);
  }

  LcmTrajectory::Trajectory force_traj =
      repos_execution_lcm_traj_.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_repos_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_repos_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputTrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_execution_lcm_traj) const {
  LcmTrajectory execution_lcm_traj =
      is_doing_c3_ ? c3_execution_lcm_traj_ : repos_execution_lcm_traj_;

  // Returned trajectory includes EE positions and feed-forward forces.
  LcmTrajectory::Trajectory end_effector_traj =
      execution_lcm_traj.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);

  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  if (adaptive_ee_tilt_) {
    LcmTrajectory::Trajectory ee_orientation_traj =
        execution_lcm_traj.GetTrajectory("end_effector_orientation_target");
    lcm_traj.AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);
  }

  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  LcmTrajectory::Trajectory force_traj =
      execution_lcm_traj.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputIsC3Mode(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  VectorXd vec = VectorXd::Constant(1, is_doing_c3_);
  MatrixXd c3_mode_data = MatrixXd::Zero(1, 1);
  VectorXd timestamp = VectorXd::Zero(1);

  // Read the boolean value into the matrix.
  c3_mode_data(0, 0) = vec(0);

  LcmTrajectory::Trajectory c3_mode;
  c3_mode.traj_name = "is_c3_mode";
  c3_mode.datatypes = vector<std::string>(1, "bool");
  c3_mode.datapoints = c3_mode_data;
  c3_mode.time_vector = timestamp.cast<double>();
  LcmTrajectory c3_mode_traj({c3_mode}, {"is_c3_mode"}, "is_c3_mode",
                             "is_c3_mode", false);

  output->saved_traj = c3_mode_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

// Output port handler for Dynamically feasible trajectory used for cost
// computation. This will directy output an lcmt_timestamped_saved_traj
// object with the dynamically feasible trajectory.
void SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_actor)
    const {
  vector<VectorXd> dynamically_feasible_traj =
      vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i] << all_sample_dynamically_feasible_plans_.at(
        SampleIndex::kCurrentLocation)[i];
  }

  MatrixXd knots = MatrixXd::Zero(3, dynamically_feasible_traj.size());
  VectorXd timestamps = VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].head(3);
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  LcmTrajectory::Trajectory ee_traj;
  MatrixXd position_samples = MatrixXd::Zero(3, N_ + 1);
  position_samples = knots.bottomRows(3);
  ee_traj.traj_name = "ee_position_target";
  ee_traj.datatypes = vector<std::string>(position_samples.rows(), "double");
  ee_traj.datapoints = position_samples;
  ee_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory ee_traj_lcm({ee_traj}, {"ee_position_target"},
                            "ee_position_target", "ee_position_target", false);

  // Include end effector orientation targets, if enabled.
  IncludeEEOrientationTargetIfEnabled(&ee_traj_lcm, ee_position_curr_,
                                      timestamps);

  dynamically_feasible_curr_plan_actor->saved_traj =
      ee_traj_lcm.GenerateLcmObject();
  dynamically_feasible_curr_plan_actor->utime = context.get_time() * 1e6;
}

// Output port handler for Dynamically feasible trajectory used for cost
// computation.
void SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_object)
    const {
  vector<VectorXd> dynamically_feasible_traj =
      vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i] << all_sample_dynamically_feasible_plans_.at(
        SampleIndex::kCurrentLocation)[i];
  }

  // Timestamps just need to be monotonically increasing, so use the index.
  VectorXd timestamps = VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  vector<LcmTrajectory::Trajectory> object_trajs;
  vector<std::string> traj_names;
  vector<MatrixXd> knots_vec;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd knots = MatrixXd::Zero(7, N_ + 1);
    for (int j = 0; j < dynamically_feasible_traj.size(); j++) {
      knots.col(j) = dynamically_feasible_traj[j].segment(3 + 7 * i, 7);
    }
    knots_vec.push_back(knots);

    LcmTrajectory::Trajectory object_traj;
    MatrixXd position_sample = MatrixXd::Zero(3, N_ + 1);
    position_sample = knots.bottomRows(3);
    object_traj.traj_name = "object_position_target_" + std::to_string(i);
    object_traj.datatypes =
        vector<std::string>(position_sample.rows(), "double");
    object_traj.datapoints = position_sample;
    object_traj.time_vector = timestamps.cast<double>();

    object_trajs.push_back(object_traj);
    traj_names.push_back("object_position_target_" + std::to_string(i));
  }

  LcmTrajectory lcm_traj(object_trajs, traj_names, "object_target",
                         "object_target", false);

  for (int i = 0; i < controller_params_.num_objects; i++) {
    LcmTrajectory::Trajectory object_orientation_traj;
    MatrixXd orientation_sample = MatrixXd::Zero(4, N_ + 1);
    orientation_sample = knots_vec.at(i).topRows(4);
    object_orientation_traj.traj_name =
        "object_orientation_target_" + std::to_string(i);
    object_orientation_traj.datatypes =
        vector<std::string>(orientation_sample.rows(), "double");
    object_orientation_traj.datapoints = orientation_sample;
    object_orientation_traj.time_vector = timestamps.cast<double>();
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           object_orientation_traj);
  }

  dynamically_feasible_curr_plan_object->saved_traj =
      lcm_traj.GenerateLcmObject();
  dynamically_feasible_curr_plan_object->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputDynamicallyFeasibleBestPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_best_plan)
    const {
  vector<VectorXd> dynamically_feasible_traj =
      vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(best_sample_index_)[i];
  }

  MatrixXd knots = MatrixXd::Zero(3, dynamically_feasible_traj.size());
  VectorXd timestamps = VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].head(3);
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  LcmTrajectory::Trajectory ee_traj;
  MatrixXd position_samples = MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  ee_traj.traj_name = "ee_position_target";
  ee_traj.datatypes = vector<std::string>(position_samples.rows(), "double");
  ee_traj.datapoints = position_samples;
  ee_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory ee_traj_lcm({ee_traj}, {"ee_position_target"},
                            "ee_position_target", "ee_position_target", false);

  // Include end effector orientation targets, if enabled.
  IncludeEEOrientationTargetIfEnabled(&ee_traj_lcm, ee_position_curr_,
                                      timestamps);

  dynamically_feasible_best_plan->saved_traj = ee_traj_lcm.GenerateLcmObject();
  dynamically_feasible_best_plan->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputDynamicallyFeasibleBestPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_best_plan)
    const {
  vector<VectorXd> dynamically_feasible_traj =
      vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(best_sample_index_)[i];
  }

  // Timestamps just need to be monotonically increasing, so use the index.
  VectorXd timestamps = VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  vector<LcmTrajectory::Trajectory> object_trajs;
  vector<std::string> traj_names;
  vector<MatrixXd> knots_vec;
  for (int i = 0; i < controller_params_.num_objects; i++) {
    MatrixXd knots = MatrixXd::Zero(7, dynamically_feasible_traj.size());
    for (int j = 0; j < dynamically_feasible_traj.size(); j++) {
      knots.col(j) = dynamically_feasible_traj[j].segment(3 + 7 * i, 7);
    }
    knots_vec.push_back(knots);

    LcmTrajectory::Trajectory object_traj;
    MatrixXd position_sample = MatrixXd::Zero(3, N_ + 1);
    position_sample = knots.bottomRows(3);
    object_traj.traj_name = "object_position_target_" + std::to_string(i);
    object_traj.datatypes =
        vector<std::string>(position_sample.rows(), "double");
    object_traj.datapoints = position_sample;
    object_traj.time_vector = timestamps.cast<double>();

    object_trajs.push_back(object_traj);
    traj_names.push_back("object_position_target_" + std::to_string(i));
  }

  LcmTrajectory lcm_traj(object_trajs, traj_names, "object_target",
                         "object_target", false);

  for (int i = 0; i < controller_params_.num_objects; i++) {
    LcmTrajectory::Trajectory object_orientation_traj;
    MatrixXd orientation_sample = MatrixXd::Zero(4, N_ + 1);
    orientation_sample = knots_vec.at(i).topRows(4);
    object_orientation_traj.traj_name =
        "object_orientation_target_" + std::to_string(i);
    object_orientation_traj.datatypes =
        vector<std::string>(orientation_sample.rows(), "double");
    object_orientation_traj.datapoints = orientation_sample;
    object_orientation_traj.time_vector = timestamps.cast<double>();
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           object_orientation_traj);
  }

  dynamically_feasible_best_plan->saved_traj = lcm_traj.GenerateLcmObject();
  dynamically_feasible_best_plan->utime = context.get_time() * 1e6;
}

// Output port handlers for sample-related ports
void SamplingC3Controller::OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_all_sample_locations) const {
  vector<Vector3d> sample_locations = vector<Vector3d>(
      all_sample_locations_.begin(), all_sample_locations_.end());
  // Pad with zeros to make sure the size is max_num_samples_ + 1 for the
  // visualizer.
  while (sample_locations.size() < max_num_samples_ + 1) {
    sample_locations.push_back(Vector3d::Zero());
  }

  MatrixXd sample_datapoints = MatrixXd::Zero(3, sample_locations.size());
  VectorXd timestamps = VectorXd::Zero(sample_locations.size());
  for (int i = 0; i < sample_locations.size(); i++) {
    sample_datapoints.col(i) = sample_locations[i];
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  LcmTrajectory::Trajectory sample_positions;
  sample_positions.traj_name = "sample_locations";
  sample_positions.datatypes = vector<std::string>(3, "double");
  sample_positions.datapoints = sample_datapoints;
  sample_positions.time_vector = timestamps.cast<double>();
  LcmTrajectory sample_traj({sample_positions}, {"sample_locations"},
                            "sample_locations", "sample_locations", false);

  output_all_sample_locations->saved_traj = sample_traj.GenerateLcmObject();
  output_all_sample_locations->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_all_sample_costs) const {
  MatrixXd cost_datapoints = MatrixXd::Zero(1, all_sample_costs_.size());
  VectorXd timestamps = VectorXd::Zero(all_sample_costs_.size());

  for (int i = 0; i < all_sample_costs_.size(); i++) {
    cost_datapoints(0, i) = all_sample_costs_[i];
    timestamps(i) = i;  // dummy (just needs to monotonically increase)
  }

  LcmTrajectory::Trajectory sample_costs_traj;
  sample_costs_traj.traj_name = "sample_costs";
  sample_costs_traj.datatypes = vector<std::string>(1, "double");
  sample_costs_traj.datapoints = cost_datapoints;
  sample_costs_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory cost_traj({sample_costs_traj}, {"sample_costs"}, "sample_costs",
                          "sample_costs", false);

  output_all_sample_costs->saved_traj = cost_traj.GenerateLcmObject();
  output_all_sample_costs->utime = context.get_time() * 1e6;

  if (verbose_) {
    std::cout << "All sample costs as per output port: " << std::endl;
    for (int i = 0; i < all_sample_costs_.size(); i++) {
      std::cout << all_sample_costs_[i] << std::endl;
    }
  }
}

void SamplingC3Controller::OutputDebug(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_sampling_c3_debug* debug_msg) const {
  debug_msg->utime = context.get_time() * 1e6;
  debug_msg->is_c3_mode = is_doing_c3_;

  // Redundant radio things included in debug message for convenience.
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  debug_msg->is_teleop = radio_out->channel[14];  // 14 = teleop
  debug_msg->is_force_tracking =
      !radio_out->channel[11];                            // 11 = force tracking
                                                          //      disabled
  debug_msg->is_forced_into_c3 = radio_out->channel[12];  // 12 = forced into C3
  debug_msg->in_pose_tracking_mode = crossed_cost_switching_threshold_;
  debug_msg->mode_switch_reason = mode_switch_reason_;
  debug_msg->source_of_pursued_target = pursued_target_source_;
  debug_msg->detected_goal_changes = detected_goal_changes_;
  debug_msg->best_progress_steps_ago = best_progress_steps_ago_;
  debug_msg->lowest_cost = lowest_cost_;
  debug_msg->lowest_pos_and_rot_current_cost = lowest_pos_and_rot_current_cost_;
  debug_msg->lowest_position_error = lowest_position_error_;
  debug_msg->lowest_orientation_error = lowest_orientation_error_;
  debug_msg->current_pos_error = current_position_error_;
  debug_msg->current_rot_error = current_orientation_error_;
}

void SamplingC3Controller::OutputSampleBufferConfigurations(
    const drake::systems::Context<double>& context,
    MatrixXd* sample_buffer_configurations) const {
  *sample_buffer_configurations = sample_buffer_;
}

void SamplingC3Controller::OutputSampleBufferCosts(
    const drake::systems::Context<double>& context,
    VectorXd* sample_buffer_costs) const {
  *sample_buffer_costs = sample_costs_buffer_;
}

void SamplingC3Controller::OutputUnsuccessfulSampleBufferConfigurations(
    const drake::systems::Context<double>& context,
    MatrixXd* unsuccessful_sample_buffer_configurations) const {
  *unsuccessful_sample_buffer_configurations = unsuccessful_sample_buffer_;
}

void SamplingC3Controller::OutputUnsuccessfulSampleBufferCosts(
    const drake::systems::Context<double>& context,
    VectorXd* unsuccessful_sample_buffer_costs) const {
  *unsuccessful_sample_buffer_costs = unsuccessful_sample_costs_buffer_;
}

}  // namespace systems
}  // namespace dairlib
