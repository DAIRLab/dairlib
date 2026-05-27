#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>
#include "lcm/lcm_trajectory.h"
#include "examples/iC3/iC3_options.h"

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include "drake/systems/framework/leaf_system.h"

using dairlib::LcmTrajectory;
using dairlib::lcmt_timestamped_saved_traj;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using drake::systems::BasicVector;
using dairlib::lcmt_radio_out;

/*
  When not tracking iC3, commands osc to hold nominal position
  We solve c3 with plate initially at origin, so also translates positions back into world frame
*/

namespace dairlib {

class TimedGate : public drake::systems::LeafSystem<double> {
 public:
  // example idx 0: plate
  // example idx 1: trifinger
  explicit TimedGate(iC3Options ic3_options, int example_idx);

  const drake::systems::InputPort<double>& get_input_port_c3_actor() const {
    return this->get_input_port(c3_actor_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_timestep() const {
    return this->get_input_port(timestep_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
    return this->get_input_port(ic3_x_trajectory_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_actor() const {
    return this->get_output_port(actor_output_port_);
  }


 private:
   void OutputActorTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex c3_actor_port_;
  drake::systems::InputPortIndex nominal_position_port_;
  drake::systems::InputPortIndex ic3_x_trajectory_port_;
  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex timestep_input_port_;

  drake::systems::OutputPortIndex actor_output_port_;

  int example_idx_;

  iC3Options ic3_options_;
  bool hold_final_position_;
  int N_;
  double dt_;

};

} // namespace dairlib