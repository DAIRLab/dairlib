#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/*
 * AffineController class that generates a control input of the form
 * u = K(x_desired - x_current) + E
 * The controller has two input ports.
 * The first port is of type OutputVector<double> and has the state information
 * of the system.
 * The second input port is of type AffineParams and has the parameters
 * (K, E and x_desired) of the controller.
 * The controller has a single output port of type TimestampedVector<double>
 * that holds the control inputs plus a timestamp that is taken from the
 * timestamp of the OutputVector input port.
 */
class AffineController : public drake::systems::LeafSystem<double> {
 public:
  AffineController(int num_positions, int num_velocities, int num_efforts);

  const drake::systems::InputPort<double>& get_input_port_params() const {
    return this->get_input_port(input_port_params_index_);
  }

  const drake::systems::InputPort<double>& get_input_port_info() const {
    return this->get_input_port(input_port_info_index_);
  }

  const drake::systems::OutputPort<double>& get_output_port_control() const {
    return this->get_output_port(output_port_control_index_);
  }

  int get_input_port_info_index() { return input_port_info_index_; }

  int get_input_port_params_index() { return input_port_params_index_; }

  int get_output_port_control_index() { return output_port_control_index_; }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  const int num_states_;
  const int num_efforts_;
  int input_port_info_index_;
  int input_port_params_index_;
  int output_port_control_index_;
};

/*
 * Class that extends TimeStampedVector to store the parameters required
 * by the affine controller.
 * The K matrix, E vector and and desired state are stored.
 * Uses Eigen Maps to map the vector data onto K, E, and desired state
 *   in this order x = [K; E; desired_state]
 * Data is best accessed via getters, get_K, get_E, and get_desired_state
 */
class AffineParams : public TimestampedVector<double> {
 public:
  AffineParams(int num_states, int num_efforts)
      : TimestampedVector<double>(num_states * num_efforts + num_efforts +
                                  num_states),
        num_states_(num_states),
        num_efforts_(num_efforts),
        K_(get_mutable_data().head(num_states * num_efforts).data(),
           num_efforts, num_states),
        E_(get_mutable_data()
               .segment(num_states * num_efforts, num_efforts)
               .data(),
           num_efforts),
        desired_state_(
            get_mutable_data()
                .segment(num_states * num_efforts + num_efforts, num_states)
                .data(),
            num_states) {}

  int get_num_states() const { return num_states_; }

  // gets K as a const reference (mutable)
  const Eigen::Map<Eigen::MatrixXd>& get_K() const { return K_; }

  // gets E as a const reference (mutable)
  const Eigen::Map<Eigen::VectorXd>& get_E() const { return E_; }

  // gets desired_state as a const reference (mutable)
  const Eigen::Map<Eigen::VectorXd>& get_desired_state() const {
    return desired_state_;
  }

  void set_K(Eigen::MatrixXd K) { K_ = K; }

  void set_E(Eigen::VectorXd E) { E_ = E; }

  void set_desired_state(Eigen::VectorXd desired_state) {
    desired_state_ = desired_state;
  }

 private:
  AffineParams* DoClone() const override {
    return new AffineParams(num_states_, num_efforts_);
  }

  int num_states_;
  int num_efforts_;
  Eigen::Map<Eigen::MatrixXd> K_;
  Eigen::Map<Eigen::VectorXd> E_;
  Eigen::Map<Eigen::VectorXd> desired_state_;
};

}  // namespace systems
}  // namespace dairlib
