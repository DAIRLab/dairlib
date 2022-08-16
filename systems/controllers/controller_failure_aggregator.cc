#include "systems/controllers/controller_failure_aggregator.h"

#include <limits>

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
    std::string controller_channel, int num_input_ports) :
    controller_channel_(controller_channel){
  this->set_name("controller_failure_aggregator");
  for (int i = 0; i < num_input_ports; ++i) {
    input_ports_.push_back(
        this->DeclareVectorInputPort("failure_signal" + std::to_string(i),
                                     TimestampedVector<double>(1))
            .get_index());
  }
  status_output_port_ = this->DeclareAbstractOutputPort(
                         "lcmt_controller_failure",
                         &ControllerFailureAggregator::AggregateFailureSignals)
                     .get_index();
}

void ControllerFailureAggregator::AggregateFailureSignals(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_controller_failure* output) const {
  int status = 0;
  double timestamp;
  for (auto port : input_ports_) {
    const TimestampedVector<double>* is_error =
        (TimestampedVector<double>*)this->EvalVectorInput(context, port);
    status = std::max(status, (int) is_error->get_value()(0));
    timestamp = is_error->get_timestamp();
  }
  output->controller_channel = controller_channel_;
  output->error_code = status;
  output->utime = timestamp * 1e6;
  output->error_name = "";
}

}  // namespace systems
}  // namespace dairlib
