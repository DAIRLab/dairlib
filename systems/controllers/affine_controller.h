#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems{

/*
AffineController class that generates a control input of the form u = K(x_desired - x_current) + E
The controller has two input ports.
The first port is of type OutputVector<double> and has the state information of the system
The second input port is of type AffineParams and has the parameters (K, E and x_desired) of the controller
The controller has a single output port of type TimestampedVector<double> that holds the control inputs plus a timestamp that is taken from the timestamp of the OutputVector input port
 */
class AffineController : public LeafSystem<double> {

  public:
    AffineController(int num_positions, int num_velocities, int num_efforts);

    const drake::systems::InputPort<double>& get_input_port_params() const
    {
      return this->get_input_port(input_port_params_index_);
    }

    const drake::systems::InputPort<double>& get_input_port_info() const 
    {
      return this->get_input_port(input_port_info_index_);
    }

    int get_input_port_info_index()
    {
        return input_port_info_index_;
    }

    int get_input_port_params_index()
    {
        return input_port_params_index_;
    }

    MatrixXd VecToMat(VectorXd v, int num_rows, int num_cols) const;
    VectorXd MatToVec(MatrixXd m) const;

  private:

    void CalcControl(const Context<double>& context,
                     TimestampedVector<double>* output) const;
    const int num_states_;
    const int num_efforts_;
    int input_port_info_index_;
    int input_port_params_index_;
};


class AffineParams : public TimestampedVector<double> {

  public:
    AffineParams(int num_states, int num_efforts):
      TimestampedVector<double>(num_states * num_efforts + num_efforts + num_states),
      num_states_(num_states), num_efforts_(num_efforts) {}

  int GetNumStates() const { return num_states_;}

  private:
  AffineParams* DoClone() const override {
    return new AffineParams(num_states_, num_efforts_);
  }

  int num_states_;
  int num_efforts_;

};

}// namespace systems
}// namespace dairlib
