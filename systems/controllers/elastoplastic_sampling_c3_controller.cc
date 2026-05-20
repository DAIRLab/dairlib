#include "systems/controllers/elastoplastic_sampling_c3_controller.h"

#include "c3/core/c3_miqp.h"
#include "c3/core/c3_plus.h"
#include "c3/core/c3_qp.h"
#include "dairlib/lcmt_elastoplastic_network.hpp"

#include "drake/common/drake_assert.h"

namespace dairlib {

using c3::C3;
using c3::C3MIQP;
using c3::C3Plus;
using c3::C3QP;
using c3::ElastoPlasticLCSFactoryOptions;
using c3::LCS;
using c3::multibody::ElastoPlasticLCSFactory;
using c3::multibody::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

namespace systems {

ElastoPlasticSC3Controller::ElastoPlasticSC3Controller(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const vector<vector<SortedPair<GeometryId>>>& external_contact_pair_lists,
    const vector<GeometryId>& internal_contact_geometries,
    DeformControllerParams controller_params, bool verbose)
    : SamplingC3Controller(
          plant, context, plant_ad, context_ad, external_contact_pair_lists,
          std::move(controller_params.sampling_c3_controller_params), verbose),
      internal_contact_geometries_(internal_contact_geometries),
      n_lambda_internal_(
          3 *
          controller_params.elastoplastic_sc3_options.num_internal_contacts),
      n_nodes_(internal_contact_geometries.size()),
      elastoplastic_sc3_options_(controller_params.elastoplastic_sc3_options) {
  this->set_name("elastoplastic_sc3_controller");

  // Edit some of the work of the parent constructor.
  dt_ = elastoplastic_sc3_options_.planning_dt_pose;  // Always in pose mode.
  G_ = std::vector<MatrixXd>(N_, elastoplastic_sc3_options_.G);
  U_ = std::vector<MatrixXd>(N_, elastoplastic_sc3_options_.U);
  // Need to rebuild C3 plan objects since they use differently sized LCS.
  auto lcs_placeholder = LCS::CreatePlaceholderLCS(
      n_x_, n_u_, n_lambda_ + n_lambda_internal_, N_, dt_);
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  C3Options c3_options = elastoplastic_sc3_options_.GetC3Options();
  if (elastoplastic_sc3_options_.projection_type == "MIQP") {
    c3_curr_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3MIQP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  } else if (elastoplastic_sc3_options_.projection_type == "QP") {
    c3_curr_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
  } else if (elastoplastic_sc3_options_.projection_type == "C3+") {
    c3_curr_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_best_plan_ = std::make_unique<C3Plus>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3Plus>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  }  // No need to throw error otherwise since done by parent constructor.
  n_z_ = c3_curr_plan_->GetZSize();
  c3_curr_plan_->SetSolverOptions(solver_options_);
  c3_best_plan_->SetSolverOptions(solver_options_);
  c3_buffer_plan_->SetSolverOptions(solver_options_);
  // Set actor bounds.
  if (!elastoplastic_sc3_options_.include_walls) {
    for (int i = 0; i < elastoplastic_sc3_options_.workspace_limits.size();
         ++i) {
      Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
      A.segment(0, 3) =
          elastoplastic_sc3_options_.workspace_limits[i].segment(0, 3);
      c3_curr_plan_->AddLinearConstraint(
          A, elastoplastic_sc3_options_.workspace_limits[i][3],
          elastoplastic_sc3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
      c3_best_plan_->AddLinearConstraint(
          A, elastoplastic_sc3_options_.workspace_limits[i][3],
          elastoplastic_sc3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
      c3_buffer_plan_->AddLinearConstraint(
          A, elastoplastic_sc3_options_.workspace_limits[i][3],
          elastoplastic_sc3_options_.workspace_limits[i][4],
          c3::ConstraintVariable::STATE);
    }
  }
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_horizontal_limits[0],
        elastoplastic_sc3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_best_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_horizontal_limits[0],
        elastoplastic_sc3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_buffer_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_horizontal_limits[0],
        elastoplastic_sc3_options_.u_horizontal_limits[1],
        c3::ConstraintVariable::INPUT);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_vertical_limits[0],
        elastoplastic_sc3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_best_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_vertical_limits[0],
        elastoplastic_sc3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
    c3_buffer_plan_->AddLinearConstraint(
        A, elastoplastic_sc3_options_.u_vertical_limits[0],
        elastoplastic_sc3_options_.u_vertical_limits[1],
        c3::ConstraintVariable::INPUT);
  }
  DeclareForcedDiscreteUpdateEvent(&ElastoPlasticSC3Controller::ComputePlan);

  // New input port for elastoplastic network.
  elastoplastic_input_port_ =
      this->DeclareAbstractInputPort(
              "elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  // Use a sampling strategy compatible with the state representation of a
  // deformable object.
  DRAKE_DEMAND(sampling_params_.sampling_strategy ==
               SamplingStrategy::kRandomOnSphereAroundDeformable);
}

// TODO @bibit
drake::systems::EventStatus ElastoPlasticSC3Controller::ComputePlan(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  return SamplingC3Controller::ComputePlan(context, discrete_state);
}

std::pair<vector<LCS>, vector<LCS>>
ElastoPlasticSC3Controller::CreateLCSObjectsForSamples(
    const vector<Eigen::VectorXd>& candidate_states,
    const drake::VectorX<double>& x_lcs_curr,
    const ElastoPlasticLCSFactoryOptions& eplcs_factory_options) const {
  // Evaluate input port to get the elastoplastic network contents.
  const auto& elastoplastic_network_lcmt =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          *context_, elastoplastic_input_port_);
  DRAKE_DEMAND(elastoplastic_network_lcmt->num_points == n_nodes_);

  // Extract the appropriate internal contact pairs and yield forces.
  int n_connections = elastoplastic_network_lcmt->num_connections;
  DRAKE_DEMAND(3 * n_connections == n_lambda_internal_);
  vector<SortedPair<GeometryId>> internal_contact_pairs;
  vector<double> yield_forces(n_connections);
  VectorXd spring_constants =
      VectorXd::Zero(n_connections);  // TODO @bibit:  unused until more
                                      // deformation models are implemented
  for (int i = 0; i < n_connections; i++) {
    GeometryId geom_1 = internal_contact_geometries_.at(
        elastoplastic_network_lcmt->connections.at(i)[0]);
    GeometryId geom_2 = internal_contact_geometries_.at(
        elastoplastic_network_lcmt->connections.at(i)[1]);
    internal_contact_pairs.push_back(SortedPair<GeometryId>(geom_1, geom_2));
    yield_forces[i] = elastoplastic_network_lcmt->yield_forces.at(i);
    spring_constants(i) = elastoplastic_network_lcmt->spring_constants.at(i);
  }

  // Build the LCS candidates.
  vector<LCS> lcs_candidates;
  vector<LCS> lcs_candidates_for_cost;

  int num_total_samples = candidate_states.size();
  for (int i = 0; i < num_total_samples; i++) {
    // Context needs to be updated to create the LCS objects.
    UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                  candidate_states[i]);

    // Resolve external contact pairs and create the LCS.
    vector<SortedPair<GeometryId>> resolved_external_contact_pairs =
        GetResolvedContactPairs(
            plant_, *context_, contact_pairs_,
            elastoplastic_sc3_options_.resolve_contacts_to,
            elastoplastic_sc3_options_.num_friction_directions_per_contact
                .value(),
            verbose_);
    LCS lcs_object_sample =
        ElastoPlasticLCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                                resolved_external_contact_pairs,
                                internal_contact_pairs, yield_forces,
                                eplcs_factory_options)
            .GenerateLCS();
    lcs_candidates.push_back(lcs_object_sample);

    // Create different LCS objects for cost calculation.
    vector<SortedPair<GeometryId>> resolved_external_contact_pairs_for_cost =
        GetResolvedContactPairs(
            plant_, *context_, contact_pairs_,
            elastoplastic_sc3_options_.resolve_contacts_to_for_cost,
            elastoplastic_sc3_options_.num_friction_directions_per_contact
                .value(),
            verbose_);
    ElastoPlasticLCSFactoryOptions eplcs_factory_options_for_cost = {
        .deformation_model = elastoplastic_sc3_options_.deformation_model,
        .num_internal_contacts =
            elastoplastic_sc3_options_.num_internal_contacts,
        .contact_model = elastoplastic_sc3_options_.contact_model,
        .N = N_ * elastoplastic_sc3_options_.lcs_dt_resolution,
        .dt = dt_ / elastoplastic_sc3_options_.lcs_dt_resolution,
        .num_contacts = resolved_external_contact_pairs_for_cost.size(),
        .spring_stiffness = 0.0,
        .num_friction_directions_per_contact =
            elastoplastic_sc3_options_
                .num_friction_directions_per_contact_for_cost,
        .mu_per_contact = elastoplastic_sc3_options_.mu_for_cost,
        .planar_normal_direction =
            elastoplastic_sc3_options_.planar_normal_direction};
    LCS lcs_object_sample_for_cost =
        ElastoPlasticLCSFactory(plant_, *context_, plant_ad_, *context_ad_,
                                resolved_external_contact_pairs_for_cost,
                                internal_contact_pairs, yield_forces,
                                eplcs_factory_options_for_cost)
            .GenerateLCS();
    lcs_candidates_for_cost.push_back(lcs_object_sample_for_cost);
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

void ElastoPlasticSC3Controller::PruneOutdatedSamplesFromBuffer(
    const Eigen::VectorXd& x_lcs, int* num_in_buffer,
    Eigen::MatrixXd* sample_buffer, Eigen::VectorXd* sample_costs_buffer,
    const double& pos_error_sample_retention,
    [[maybe_unused]] const double& ang_error_sample_retention) const {
  (void)ang_error_sample_retention;  // Unused; network nodes are 3D only.

  int n_buffer_length = sample_costs_buffer->size();

  // Get elastoplastic network node locations, both current and from the
  // buffer.
  vector<Eigen::Array<bool, Eigen::Dynamic, 1>> mask_satisfies_pos;
  for (int i = 0; i < n_nodes_; i++) {
    Vector3d node_curr = x_lcs.segment(3 + 3 * i, 3);
    MatrixXd node_buffer =
        sample_buffer->block(0, 3 + 3 * i, n_buffer_length, 3);

    // Compute the linear difference.
    MatrixXd pos_deltas = node_buffer.rowwise() - node_curr.transpose();
    VectorXd distances = pos_deltas.rowwise().norm();
    mask_satisfies_pos.push_back(distances.array() <
                                 pos_error_sample_retention);
  }

  // Keep buffer if none of nodes moved.
  int retained_count = 0;
  MatrixXd retained_samples = MatrixXd::Zero(n_buffer_length, n_q_);
  VectorXd retained_costs = -1 * VectorXd::Ones(n_buffer_length);
  for (int i = 0; i < *num_in_buffer; i++) {
    bool keep = true;
    for (int j = 0; j < n_nodes_; j++) {
      if (!mask_satisfies_pos.at(j)[i]) {
        keep = false;
        break;
      }
    }
    if (keep) {
      retained_samples.row(retained_count) = sample_buffer->row(i);
      retained_costs[retained_count] = (*sample_costs_buffer)[i];
      retained_count++;
    }
    if ((*sample_costs_buffer)[i] < 0) {
      break;
    }
  }
  *num_in_buffer = retained_count;
  *sample_buffer = retained_samples;
  *sample_costs_buffer = retained_costs;
}

}  // namespace systems
}  // namespace dairlib
