//
// Created by yangwill on 11/15/19.
//
#pragma once

#include <string>
#include <vector>
#include <memory>

#include "common/find_resource.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

#include "drake/multibody/rigid_body_tree_construction.h"

#include "systems/robot_lcm_systems.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "systems/primitives/subvector_pass_through.h"

#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"
#include "systems/sensors/sim_cassie_sensor_aggregator.h"

class SimulatorDrift : public drake::systems::LeafSystem<double> {

 public:
  SimulatorDrift(const drake::multibody::MultibodyPlant<double>& plant,
                 const Eigen::MatrixXd& drift_rate);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcAdjustedState(const drake::systems::Context<double>& context,
                         dairlib::systems::OutputVector<double>* output) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int accumulated_drift_index_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  Eigen::MatrixXd drift_rate_;
  int state_port_;
};

