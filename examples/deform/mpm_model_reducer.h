#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_elastoplastic_network.hpp"
#include "dairlib/lcmt_tetrahedra.hpp"
#include "examples/deform/parameter_headers/reduced_model_params.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts MPM points to LCM type lcmt_tetrahedra.
class MpmPointsToTetrahedra : public drake::systems::LeafSystem<double> {
 public:
  MpmPointsToTetrahedra(ReducedModelParams reduction_params);

  const drake::systems::InputPort<double>& get_input_port_lcmt_material_points()
      const {
    return this->get_input_port(lcmt_material_points_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcmt_tetrahedra()
      const {
    return this->get_output_port(lcmt_tetrahedra_output_port_);
  }

 private:
  void OutputTetrahedraLcm(const drake::systems::Context<double>& context,
                           dairlib::lcmt_tetrahedra* output) const;
  std::pair<Eigen::Matrix3Xd, Eigen::Matrix4Xi>
  ComputeTetrahedraFromSupportFunction(
      const Eigen::Matrix3Xd& mpm_points) const;

  const ReducedModelTypes reduction_type_;
  const int n_support_directions_;
  const Eigen::Matrix3Xd support_directions_;
  const int n_fixed_tetrahedra_;
  const Eigen::Matrix4Xi fixed_tetrahedra_;

  drake::systems::InputPortIndex lcmt_material_points_input_port_;
  drake::systems::OutputPortIndex lcmt_tetrahedra_output_port_;
};

/// Converts tetrahedra representing an approximately volumetrically homogeneous
/// solid to LCM type lcmt_elastoplastic_network.
class TetrahedraToElastoPlasticNetwork
    : public drake::systems::LeafSystem<double> {
 public:
  TetrahedraToElastoPlasticNetwork(
      double youngs_modulus, double yield_stress, double mass,
      SpringConstantMethods spring_constant_method);

  const drake::systems::InputPort<double>& get_input_port_lcmt_tetrahedra()
      const {
    return this->get_input_port(lcmt_tetrahedra_input_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcmt_elastoplastic_network() const {
    return this->get_output_port(lcmt_elastoplastic_network_output_port_);
  }

 private:
  void OutputElastoPlasticNetworkLcm(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_elastoplastic_network* output) const;

  const double youngs_modulus_;
  const double yield_stress_;
  const double mass_;
  const SpringConstantMethods spring_constant_method_;

  drake::systems::InputPortIndex lcmt_tetrahedra_input_port_;
  drake::systems::OutputPortIndex lcmt_elastoplastic_network_output_port_;
};

}  // namespace systems
}  // namespace dairlib
