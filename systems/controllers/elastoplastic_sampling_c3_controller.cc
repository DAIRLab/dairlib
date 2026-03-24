#include "systems/controllers/elastoplastic_sampling_c3_controller.h"

#include "dairlib/lcmt_elastoplastic_network.hpp"
#include "solvers/elastoplastic_lcs_factory.h"

#include "drake/common/drake_assert.h"

namespace dairlib {

using drake::SortedPair;
using drake::geometry::GeometryId;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::LCS;
using solvers::LCSFactory;
using solvers::PlasticNetworkLCSFactory;
using std::vector;

namespace systems {

// TODO rework for EPSC3 (define internal_geometries_, n_lambda_internal_,
// n_nodes_, etc.)
ElastoPlasticSC3Controller::ElastoPlasticSC3Controller(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms,
    const vector<GeometryId>& internal_geometries,
    DeformControllerParams controller_params, bool verbose)
    : SamplingC3Controller(
          plant, context, plant_ad, context_ad, contact_geoms,
          std::move(controller_params.sampling_c3_controller_params), verbose) {
}

// TODO rework for EPSC3
drake::systems::EventStatus ElastoPlasticSC3Controller::ComputePlan(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  return SamplingC3Controller::ComputePlan(context, discrete_state);
}

LCS ElastoPlasticSC3Controller::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_ + n_lambda_internal_);
  MatrixXd E = MatrixXd::Zero(n_lambda_ + n_lambda_internal_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_ + n_lambda_internal_,
                              n_lambda_ + n_lambda_internal_);
  MatrixXd H = MatrixXd::Zero(n_lambda_ + n_lambda_internal_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_ + n_lambda_internal_);
  return LCS(A, B, D, d, E, F, H, c, elastoplastic_sc3_options_.N,
             elastoplastic_sc3_options_.planning_dt_position);
}

std::pair<vector<LCS>, vector<LCS>>
ElastoPlasticSC3Controller::CreateLCSObjectsForSamples(
    const vector<Eigen::VectorXd>& candidate_states,
    const drake::VectorX<double>& x_lcs_curr, const C3Options& c3_options,
    const C3Options& c3_options_curr_location) const {
  // Evaluate input port to get the elastoplastic network contents.
  const auto& elastoplastic_network_lcmt =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          *context_, elastoplastic_input_port_);
  DRAKE_DEMAND(elastoplastic_network_lcmt->num_points == n_nodes_);

  // Extract the appropriate internal contact pairs and yield forces.
  int n_connections = elastoplastic_network_lcmt->num_connections;
  vector<SortedPair<GeometryId>> internal_contact_pairs;
  VectorXd yield_forces = VectorXd::Zero(n_connections);
  VectorXd spring_constants = VectorXd::Zero(n_connections);
  for (int i = 0; i < n_connections; i++) {
    GeometryId geom_1 = internal_geometries_.at(
        elastoplastic_network_lcmt->connections.at(i)[0]);
    GeometryId geom_2 = internal_geometries_.at(
        elastoplastic_network_lcmt->connections.at(i)[1]);
    internal_contact_pairs.push_back(SortedPair<GeometryId>(geom_1, geom_2));
    yield_forces(i) = elastoplastic_network_lcmt->yield_forces.at(i);
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

    // Resolve the contact pairs and create the LCS.
    vector<SortedPair<GeometryId>> resolved_contact_pairs =
        LCSFactory::PreProcessor(plant_, *context_, contact_pairs_,
                                 elastoplastic_sc3_options_.resolve_contacts_to,
                                 c3_options.num_friction_directions, verbose_);

    if (elastoplastic_sc3_options_.deformable_model_type ==
        DeformableModelType::kPlastic) {
      LCS lcs_object_sample = PlasticNetworkLCSFactory::ToLCS(
          plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
          internal_contact_pairs, yield_forces, c3_options.mu, dt_, N_,
          elastoplastic_sc3_options_.n_lambda_with_tangential,
          elastoplastic_sc3_options_.num_friction_directions_per_contact,
          elastoplastic_sc3_options_
              .starting_index_per_contact_in_lambda_t_vector,
          contact_model_);
      lcs_candidates.push_back(lcs_object_sample);
    } else {
      throw std::runtime_error(
          "Unimplemented or unsupported deformable model type for "
          "ElastoPlasticSC3Controller.");
    }

    // Create different LCS objects for cost calculation.
    vector<SortedPair<GeometryId>> resolved_contact_pairs_for_cost_simulation;
    resolved_contact_pairs_for_cost_simulation = LCSFactory::PreProcessor(
        plant_, *context_, contact_pairs_,
        elastoplastic_sc3_options_.resolve_contacts_to_for_cost,
        elastoplastic_sc3_options_.num_friction_directions, verbose_);
    if (elastoplastic_sc3_options_.deformable_model_type ==
        DeformableModelType::kPlastic) {
      LCS lcs_object_sample_for_cost_simulation =
          PlasticNetworkLCSFactory::ToLCS(
              plant_, *context_, plant_ad_, *context_ad_,
              resolved_contact_pairs_for_cost_simulation,
              internal_contact_pairs, yield_forces,
              elastoplastic_sc3_options_.mu_for_cost, dt_cost_,
              N_ * elastoplastic_sc3_options_.lcs_dt_resolution,
              elastoplastic_sc3_options_.n_lambda_with_tangential_cost,
              elastoplastic_sc3_options_
                  .num_friction_directions_per_contact_cost,
              elastoplastic_sc3_options_
                  .starting_index_per_contact_in_lambda_t_vector_cost,
              contact_model_);
      lcs_candidates_for_cost.push_back(lcs_object_sample_for_cost_simulation);
    } else {
      throw std::runtime_error(
          "Unimplemented or unsupported deformable model type for "
          "ElastoPlasticSC3Controller.");
    }
  }

  // Reset the context to the current lcs state.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);

  if (verbose_) {
    // Print the LCS matrices for verbose inspection.
    LCS verbose_lcs = lcs_candidates.at(SampleIndex::kCurrentLocation);
    std::cout << "A: " << std::endl;
    std::cout << verbose_lcs.A_[0] << std::endl;
    std::cout << "B: " << std::endl;
    std::cout << verbose_lcs.B_[0] << std::endl;
    std::cout << "D: " << std::endl;
    std::cout << verbose_lcs.D_[0] << std::endl;
    std::cout << "d: " << std::endl;
    std::cout << verbose_lcs.d_[0] << std::endl;
    std::cout << "E: " << std::endl;
    std::cout << verbose_lcs.E_[0] << std::endl;
    std::cout << "F: " << std::endl;
    std::cout << verbose_lcs.F_[0] << std::endl;
    std::cout << "H: " << std::endl;
    std::cout << verbose_lcs.H_[0] << std::endl;
    std::cout << "c: " << std::endl;
    std::cout << verbose_lcs.c_[0] << std::endl;
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
