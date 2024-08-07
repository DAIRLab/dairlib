#pragma once
#include <concepts>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/diagram.h"

template <class C>
concept mpfc = requires(C mpc) {
  {
    std::constructible_from<C,
      const drake::multibody::MultibodyPlant<double>&,
      drake::systems::Context<double>*,
      std::vector<int>,
      std::vector<int>,
      std::vector<std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>,
      const std::string&, const std::string&>
  };
  { mpc.MakeDrivenByStandaloneSimulator(0.01)};
  { mpc.get_input_port_state() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { mpc.get_input_port_vdes() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { mpc.get_input_port_footholds() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { mpc.get_input_port_elevation() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { mpc.get_output_port_mpc_output() } -> std::same_as<const drake::systems::OutputPort<double>&>;
  { mpc.get_output_port_mpc_debug() } -> std::same_as<const drake::systems::OutputPort<double>&>;
  { C::empty_debug_message() };
  { C::empty_output_message() };
};

template <class C>
concept mpfc_osc_diagram = requires (C diagram) {
  {
    std::constructible_from<C,
      const drake::multibody::MultibodyPlant<double>&,
      const std::string&,
      const std::string&,
      const std::string&>
  };
  { std::is_base_of_v<C, drake::systems::Diagram<double>> };
  { diagram.get_input_port_mpc_output() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { diagram.get_input_port_radio() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { diagram.get_input_port_state() } -> std::same_as<const drake::systems::InputPort<double>&>;
  { diagram.get_output_port_actuation() } -> std::same_as<const drake::systems::OutputPort<double>&>;
  { diagram.get_output_port_u_lcm() } -> std::same_as<const drake::systems::OutputPort<double>&>;
  { diagram.get_output_port_osc_debug() } -> std::same_as<const drake::systems::OutputPort<double>&>;


};