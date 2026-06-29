#pragma once

#include "c3/multibody/elastoplastic_lcs_factory_options.h"
#include "dairlib/lcmt_elastoplastic_network.hpp"
#include "examples/deform/parameter_headers/deform_controller_params.h"
#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "examples/deform/parameter_headers/goal_params.h"
#include "systems/controllers/sampling_based_c3_controller.h"

namespace dairlib {
namespace systems {

using c3::ElastoPlasticLCSFactoryOptions;

class ElastoPlasticSC3Controller : public SamplingC3Controller {
 public:
  // TODO @bibit:  should internal contact geometries already come in pairs?
  // Or should the code change to have adaptable graph structures based on the
  // current states of the deformable?
  explicit ElastoPlasticSC3Controller(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<
          std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
          external_contact_pair_lists,
      const std::vector<drake::geometry::GeometryId>&
          internal_contact_geometries,
      DeformControllerParams controller_params, bool verbose = false);

  // Additional elastoplastic input port.
  const drake::systems::InputPort<double>& get_input_port_elastoplastic()
      const {
    return this->get_input_port(elastoplastic_input_port_);
  }

 private:
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const override;

  std::pair<double, std::vector<Eigen::VectorXd>> CalcCost(
      C3CostComputationType cost_type, const c3::LCS& lcs_for_cost,
      const c3::C3::CostMatrices& cost_mats,
      const std::shared_ptr<c3::C3>& c3_object,
      const bool& force_tracking_disabled,
      const bool& print_cost_breakdown) const;

  // Don't override the parent class version, since they differ in input
  // arguments.  Instead, keep the parent and child versions private from each
  // other.
  std::pair<std::vector<c3::LCS>, std::vector<c3::LCS>>
  CreateLCSObjectsForSamples(
      const std::vector<Eigen::VectorXd>& candidate_states,
      const drake::VectorX<double>& x_lcs_curr,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
          internal_contact_pairs,
      const std::vector<double>& yield_forces,
      const ElastoPlasticLCSFactoryOptions& eplcs_factory_options) const;

  void PruneOutdatedSamplesFromBuffer(
      const Eigen::VectorXd& x_lcs, int* num_in_buffer,
      Eigen::MatrixXd* sample_buffer, Eigen::VectorXd* sample_costs_buffer,
      const double& pos_error_sample_retention,
      [[maybe_unused]] const double& ang_error_sample_retention) const override;

  // Don't override the parent class version, since they differ in input
  // arguments (only by name).  Instead, keep the parent and child versions
  // private from each other.
  void KeepTrackOfC3ModeProgress(const drake::VectorX<double>& x_lcs_curr,
                                 const BasicVector<double>& x_lcs_final_des,
                                 bool& met_minimum_progress,
                                 const bool& print_current_node_cost) const;

  void ResetProgressMetrics() const;

  std::pair<std::vector<drake::SortedPair<drake::geometry::GeometryId>>,
            std::vector<double>>
  GetCurrentElastoPlasticProperties(const dairlib::lcmt_elastoplastic_network&
                                        elastoplastic_network_lcmt) const;

  drake::systems::InputPortIndex elastoplastic_input_port_;
  const int n_lambda_internal_;
  const int n_nodes_;
  const std::vector<drake::geometry::GeometryId> internal_contact_geometries_;
  const ElastoPlasticGoalParams goal_params_;
  const ElastoPlasticSC3Options elastoplastic_sc3_options_;

  // Variables specific for elastoplastic controller that replace the parent
  // class's variables of similar purposes.
  mutable double lowest_node_current_cost_;  // lowest_pos_and_rot_current_cost_
  mutable double lowest_node_error_;         // lowest_position_error_
  mutable double current_node_error_;        // current_position_error_
  mutable std::queue<double> node_cost_history_;  // object_config_cost_history_
};

}  // namespace systems
}  // namespace dairlib
