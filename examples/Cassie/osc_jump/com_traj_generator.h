#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::Cassie::osc_jump {

class COMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  COMTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          feet_contact_points,
      drake::trajectories::PiecewisePolynomial<double> crouch_traj,
      double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> generateBalanceTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double d) const;
  drake::trajectories::PiecewisePolynomial<double> generateCrouchTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double d) const;
  drake::trajectories::PiecewisePolynomial<double> generateLandingTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double d) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  int fsm_idx_;
  int com_x_offset_idx_;

  // A list of pairs of contact body frame and contact point
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      feet_contact_points_;

  drake::trajectories::PiecewisePolynomial<double> crouch_traj_;
  double time_offset_;

  int state_port_;
  int fsm_port_;

  static constexpr double kTransitionSpeed = 20.0; // 20 m/s
  // The trajectory optimization solution sets the final CoM very close to
  // rear toe contacts - this is an offset to move it closer to the center of
  // the support polygon
  static constexpr double kLandingOffset = 0.025; // 0.025 m (2.5cm)
};

}  // namespace dairlib::examples::Cassie::osc_jump
