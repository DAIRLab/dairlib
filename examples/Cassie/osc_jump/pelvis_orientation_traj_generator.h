#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::Cassie::osc_jump {

/// General purpose leaf system that returns a trajectory segment at a
/// particular time. Need the plant in order to get in the current simulator
/// state.
class PPolyPassthrough
    : public drake::systems::LeafSystem<double> {
 public:
  PPolyPassthrough(const drake::multibody::MultibodyPlant<double>& plant,
                                 const drake::trajectories::PiecewisePolynomial<
                                     double>& orientation_traj,
                                 std::string traj_name,
                                 double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
//  const drake::systems::InputPort<double>& get_fsm_input_port() const {
//    return this->get_input_port(fsm_port_);
//  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::trajectories::PiecewisePolynomial<double> traj_;

  int state_port_;
//  int fsm_port_;
};

}  // namespace dairlib::examples::Cassie::osc_jump
