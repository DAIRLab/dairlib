#pragma once

#include <string>
#include <vector>

#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and outputs it as a drake PiecewisePolynomial.
class TrifingerPositionSplitter : public drake::systems::LeafSystem<double> {
 public:
  explicit TrifingerPositionSplitter(std::string trajectory_name);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_finger_0() const {
    return this->get_output_port(finger_0_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_finger_120() const {
    return this->get_output_port(finger_120_output_port_);
  }  
  const drake::systems::OutputPort<double>& get_output_port_finger_240() const {
    return this->get_output_port(finger_240_output_port_);
  }

 private:
  /*
  These just pass an index into output trajectory to determine which finger
  Since the output type needs to be a drake::trajectories::Trajectory, which is an abstract
    type, you can't bind the index directly using a lambda (can't allocate an abstract type)
  Since trifinger has a fixed number of fingers this is the only way I could get it to work
  */ 
  void OutputTrajectory0(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;
  void OutputTrajectory120(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;
  void OutputTrajectory240(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;      
                                          
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj, int finger_idx) const;

  drake::systems::InputPortIndex trajectory_input_port_;
  drake::systems::OutputPortIndex finger_0_output_port_;
  drake::systems::OutputPortIndex finger_120_output_port_;
  drake::systems::OutputPortIndex finger_240_output_port_;

  const std::string trajectory_name_;
};

}  // namespace dairlib
