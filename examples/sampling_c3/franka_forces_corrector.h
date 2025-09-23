#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "common/update_context.h"
#include "dairlib/lcmt_c3_forces.hpp"
#include "dairlib/lcmt_c3_output.hpp"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

// This LeafSystem will perform the following
// 1. Subscribe to the LCM channel of C3 debug info
// 2. Obtain the current lcs state and perform collision checking to get the
// contact points and force basis
// 3. Obtain `lambda_sol` from the C3 debug info and compute the forces in the
// world frame
// 4. Publish the corrected forces (net forces and contact points) as
// lcmt_c3_forces
class FrankaForcesCorrector : public drake::systems::LeafSystem<double> {
 public:
  FrankaForcesCorrector();

  const drake::systems::InputPort<double>& get_input_port_c3_debug() const {
    return this->get_input_port(lcm_c3_solution_input_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_corrected_forces() const {
    return this->get_output_port(lcm_c3_corrected_forces_output_port_);
  }

 private:
  void OutputCorrectedC3Forces(const drake::systems::Context<double>& context,
                               dairlib::lcmt_c3_forces* c3_forces_output) const;
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, int64_t> ExtractLCSStateAndForces(
      const lcmt_c3_output& c3_solution) const;

  drake::systems::InputPortIndex lcm_c3_solution_input_port_;
  drake::systems::OutputPortIndex lcm_c3_corrected_forces_output_port_;

  SamplingC3ControllerParams controller_params_;
  SamplingC3Options sampling_c3_options_;
  C3Options c3_options_;

  drake::multibody::MultibodyPlant<double>* plant_lcs_;
  std::unique_ptr<drake::systems::Diagram<double>> plant_lcs_diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  drake::systems::Context<double>* plant_lcs_context_;
  std::unique_ptr<drake::multibody::MultibodyPlant<drake::AutoDiffXd>>
      plant_lcs_ad_;
  std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> context_ad_;

  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_pairs_;
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms_;
  int n_q_{0};
  int n_v_{0};
  int n_u_{0};
  int n_x_{0};
  double dt_{0.0};
  int N_{0};
  bool verbose_{false};

  solvers::ContactModel contact_model_ = solvers::ContactModel::kAnitescu;
};

}  // namespace systems
}  // namespace dairlib