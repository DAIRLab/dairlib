#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
#include "jumping_event_based_fsm.h"

namespace dairlib::examples::Cassie::osc_jump {

class COMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  COMTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                   int pelvis_idx,
                   Eigen::Vector3d front_contact_disp,
                   Eigen::Vector3d rear_contact_disp,
                   drake::trajectories::PiecewisePolynomial<double> crouch_traj,
                   double height = 0.7009,
                   double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> generateBalanceTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x) const;
  drake::trajectories::PiecewisePolynomial<double> generateCrouchTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x) const;
  drake::trajectories::PiecewisePolynomial<double> generateFlightTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x)
      const;
  drake::trajectories::PiecewisePolynomial<double> generateLandingTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  int time_idx_;
  int fsm_idx_;
  int com_x_offset_idx_;

  int hip_idx_;
  Eigen::Vector3d front_contact_disp_;
  Eigen::Vector3d rear_contact_disp_;
  drake::trajectories::PiecewisePolynomial<double> crouch_traj_;
  double height_;

  int state_port_;
  int fsm_port_;
};

}  // namespace dairlib::examples::Cassie::osc_jump
