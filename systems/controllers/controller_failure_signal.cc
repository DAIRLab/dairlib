#include "systems/controllers/controller_failure_signal.h"

#include <limits>

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

namespace dairlib {
namespace systems {

ControllerFailureAggregator::ControllerFailureAggregator(
    std::string controller_channel_name, int num_input_ports) {
  this->set_name("controller_failure_aggregator");
  for (int i = 0; i < num_input_ports; ++i) {
    input_ports_.push_back(
        this->DeclareVectorInputPort("failure_signal" + std::to_string(i),
                                     BasicVector<double>(1))
            .get_index());
  }
  output_port_ = this->DeclareAbstractOutputPort(
                         "lcmt_controller_failure",
                         &ControllerFailureAggregator::AggregateFailureSignals)
                     .get_index();
}

void ControllerFailureAggregator::AggregateFailureSignals(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_controller_failure* output) const {}

}  // namespace systems
}  // namespace dairlib
