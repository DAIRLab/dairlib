#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/*
 * A simple joint level linear controller. Very similar to the LinearController
 * class except it takes in a state_traj and input_traj. This allows for a
 * feedforward input term and without having to manipulate the PD config
 * channel.
 */
class JointLevelController : public drake::systems::LeafSystem<double> {
 public:
  /*
   * The constructor assumes K has been constructed correctly.
   * This means that there is only feedback on the actuated joints
   */
  JointLevelController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::trajectories::PiecewisePolynomial<double>& state_traj,
      const drake::trajectories::PiecewisePolynomial<double>& input_traj,
      const Eigen::MatrixXd& K);

  /*
   * Function to get the input port that takes in the current state
   * information.
   */
  const drake::systems::InputPort<double>& get_input_port_info() const {
    return this->get_input_port(input_port_info_index_);
  }
  /*
   * Function to get the output port that outputs the computed control inputs to
   * the actuators.
   */
  const drake::systems::OutputPort<double>& get_output_port_efforts() const {
    return this->get_output_port(output_port_efforts_index_);
  }

  /*
   * Function to get the index of the input port that takes in the current state
   * information.
   */
  int get_input_port_info_index() { return input_port_info_index_; }
  /*
   * Function to get the index of the output port that outputs the computed
   * control inputs to the actuators.
   */
  int get_output_port_efforts_index() { return output_port_efforts_index_; }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* control) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  int input_port_info_index_;
  int output_port_efforts_index_;
  int time_shift_idx_;
  drake::trajectories::PiecewisePolynomial<double> state_traj_;
  drake::trajectories::PiecewisePolynomial<double> input_traj_;
  Eigen::MatrixXd K_;
  int nq_;
  int nv_;
  int nu_;
};

}  // namespace systems
}  // namespace dairlib
