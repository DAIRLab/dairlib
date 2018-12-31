#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib{
namespace systems{

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

    /*
     * Converts the given Eigen vector to an Eigen Matrix.
     * The Matrix is build column-wise.
     *
     * @param v The Eigen vector.
     * @param num_rows Number of rows of the output matrix.
     * @param num_cols Number of columns of the output matrix.
     * @throws std::runtime error if the dimensions of the required matrix
     * are incorrect.
     * Returns the constructed matrix.
     */
    static Eigen::MatrixXd VecToMat(Eigen::VectorXd v,
                             int num_rows,
                             int num_cols);
    /*
     * Converts the given Eigen matrix into a flattened Eigen Vector.
     * The vector is formed by stacking the columns of the matrix.
     *
     * @param m The Eigen matrix.
     * Returns the flattened vector.
     */
    static Eigen::VectorXd MatToVec(Eigen::MatrixXd m);

  private:

    void CalcControl(const drake::systems::Context<double>& context,
                     TimestampedVector<double>* output) const;

    const int num_states_;
    const int num_efforts_;
    int input_port_info_index_;
    int input_port_params_index_;

};

/*
 * Class that extends TimeStampedVector to store the parameters required
 * by the affine controller.
 * The K matrix, E vector and and desired state are stored.
 */
class AffineParams : public TimestampedVector<double> {

  public:
    AffineParams(int num_states, int num_efforts):
      TimestampedVector<double>(
          num_states * num_efforts + num_efforts + num_states),
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
