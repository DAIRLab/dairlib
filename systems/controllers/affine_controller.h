#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems{

class AffineController : public LeafSystem<double>
{

    public:

    AffineController(int num_positions, int num_velocities, int num_efforts);

    const drake::systems::InputPortDescriptor<double>& get_input_port_params() const
    {
      return this->get_input_port(input_port_params_index_);
    }

    const drake::systems::InputPortDescriptor<double>& get_input_port_info() const 
    {
      return this->get_input_port(input_port_info_index_);
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


class AffineParams : public TimestampedVector<double>
{

    public:

    AffineParams(int num_states, int num_efforts):
        TimestampedVector<double>(num_states * num_efforts + num_efforts + num_states),
        num_states_(num_states), num_efforts_(num_efforts) {};

    private:
    
    AffineParams* DoClone() const override {
      return new AffineParams(num_states_, num_efforts_);
    }

    int num_states_;
    int num_efforts_;

};

}// namespace systems
}// namespace dairlib
