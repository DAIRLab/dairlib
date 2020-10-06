#include "examples/Cassie/osc/timestamped_vector_adder.h"

#include <algorithm>  // std::max
#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using dairlib::systems::TimestampedVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

namespace dairlib {
namespace cassie {
namespace osc {

TimestampedVectorAdder::TimestampedVectorAdder(int num_input_ports,
                                               int port_size) {
  num_input_ports_ = num_input_ports;

  // Input/Output Setup
  for (int i = 0; i < num_input_ports; i++) {
    this->DeclareVectorInputPort(systems::TimestampedVector<double>(port_size));
  }
  this->DeclareVectorOutputPort(systems::TimestampedVector<double>(port_size),
                                &TimestampedVectorAdder::Add);
}

void TimestampedVectorAdder::Add(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* output) const {
  auto input0 =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
  drake::VectorX<double> data = input0->get_data();

  for (int i = 1; i < num_input_ports_; i++) {
    auto inputi =
        (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
    data += inputi->get_data();
  }

  output->SetDataVector(data);
  output->set_timestamp(input0->get_timestamp());
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
