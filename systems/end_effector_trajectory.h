/**This system is used by the osc diagram to read an end effector trajectory from an lcm message and pass it onto the OSC class.
* This system is also present under examples/franka/systems but has been copied here to be used by the instance of the osc
* in the sampling_c3 controller examples. */
#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class EndEffectorTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  EndEffectorTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const Eigen::VectorXd& neutral_pose,
    bool teleop_neutral_pose,
    const std::string& end_effector_name);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  void SetRemoteControlParameters(const Eigen::Vector3d& neutral_pose,
                                  double x_scale, double y_scale,
                                  double z_scale);

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcNeutralPoseBasedTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const;

  void CalcPoseShiftingTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex trajectory_port_;
  drake::systems::InputPortIndex radio_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::string end_effector_name_;
  Eigen::Vector3d neutral_pose_ = {0.55, 0, 0.40};
  mutable Eigen::Vector3d shifting_pose_ = {0.55, 0, 0.40};
  mutable bool was_in_teleop_mode_ = false;
  double x_scale_;
  double y_scale_;
  double z_scale_;
};

}  // namespace dairlib
