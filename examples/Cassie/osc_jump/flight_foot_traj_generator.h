#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc_jump {

class FlightFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FlightFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, const std::string& hip_name,
      bool isLeftFoot,
      const drake::trajectories::PiecewisePolynomial<double>& foot_traj,
      const drake::trajectories::PiecewisePolynomial<double>& hip_traj,
      double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> GenerateFlightTraj(
      const Eigen::VectorXd& x, double t) const;
  drake::trajectories::PiecewisePolynomial<double> GenerateRelativeTraj() const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::Frame<double>& world_;
  const drake::multibody::Frame<double>& hip_frame_;

  drake::trajectories::PiecewisePolynomial<double> foot_traj_;
  drake::trajectories::PiecewisePolynomial<double> hip_traj_;

  int state_port_;
  int fsm_port_;
};

}  // namespace dairlib::examples::osc_jump
