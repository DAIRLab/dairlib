#pragma once

#include "examples/deform/parameter_headers/deform_controller_params.h"
#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "systems/controllers/sampling_based_c3_controller.h"

namespace dairlib {
namespace systems {

class ElastoPlasticSC3Controller : public SamplingC3Controller {
 public:
  explicit ElastoPlasticSC3Controller(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<
          std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
          contact_geoms,
      const std::vector<drake::geometry::GeometryId>& internal_geometries,
      DeformControllerParams controller_params, bool verbose = false);

  // Additional elastoplastic input port.
  const drake::systems::InputPort<double>& get_input_port_elastoplastic()
      const {
    return this->get_input_port(elastoplastic_input_port_);
  }

 protected:
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const override;

  solvers::LCS CreatePlaceholderLCS() const override;

  std::pair<std::vector<solvers::LCS>, std::vector<solvers::LCS>>
  CreateLCSObjectsForSamples(
      const std::vector<Eigen::VectorXd>& candidate_states,
      const drake::VectorX<double>& x_lcs_curr, const C3Options& c3_options,
      const C3Options& c3_options_curr_location) const override;

  void PruneOutdatedSamplesFromBuffer(
      const Eigen::VectorXd& x_lcs, int* num_in_buffer,
      Eigen::MatrixXd* sample_buffer, Eigen::VectorXd* sample_costs_buffer,
      const double& pos_error_sample_retention,
      [[maybe_unused]] const double& ang_error_sample_retention) const override;

 private:
  drake::systems::InputPortIndex elastoplastic_input_port_;
  int n_lambda_internal_;
  int n_nodes_;
  std::vector<drake::geometry::GeometryId> internal_geometries_;
  ElastoPlasticSC3Options elastoplastic_sc3_options_;
};

}  // namespace systems
}  // namespace dairlib
