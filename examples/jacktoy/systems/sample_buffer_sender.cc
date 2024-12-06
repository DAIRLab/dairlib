#include "sample_buffer_sender.h"

#include <algorithm>
#include <iostream>
#include "common/eigen_utils.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

SampleBufferSender::SampleBufferSender(int buffer_size, int n_config) {
  this->set_name("sample_buffer_sender");

  MatrixXd sample_buffer = MatrixXd::Zero(buffer_size, n_config);
  VectorXd cost_buffer = VectorXd::Zero(buffer_size);
  samples_port_ = this->DeclareAbstractInputPort(
      "sample_buffer_configurations",
      drake::Value<MatrixXd>{sample_buffer}
    ).get_index();
  sample_costs_port_ = this->DeclareAbstractInputPort(
      "sample_buffer_costs",
      drake::Value<VectorXd>{cost_buffer}
    ).get_index();

  lcm_sample_buffer_output_port_ = this->DeclareAbstractOutputPort(
      "lcmt_sample_buffer",
      dairlib::lcmt_sample_buffer(),
      &SampleBufferSender::OutputSampleBufferLcm)
    .get_index();
}

void SampleBufferSender::OutputSampleBufferLcm(
  const drake::systems::Context<double>& context,
  dairlib::lcmt_sample_buffer* output) const {

  // Evaluate input ports to get the sample configurations and costs.
  const auto& buffer_configurations =
    this->EvalInputValue<MatrixXd>(context, samples_port_);
  const auto& buffer_costs =
    this->EvalInputValue<VectorXd>(context, sample_costs_port_);

  // Get some useful quantities.
  int n_buffer = buffer_costs->size();
  int n_config = buffer_configurations->row(0).size();
  int n_in_buffer = std::count_if(
    buffer_costs->begin(),
    buffer_costs->end(),
    [](double cost) {return cost >= 0;}
  );

  std::vector<float> cost_data(buffer_costs->data(),
                                buffer_costs->data() + n_buffer);
  std::vector<std::vector<float>> config_data;
  for (int i = 0; i < n_buffer; i++) {
    config_data.push_back(
      std::vector<float>(buffer_configurations->row(i).data(),
                         buffer_configurations->row(i).data() + n_config)
    );
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->buffer_length = n_buffer;
  output->num_configurations = n_config;
  output->num_in_buffer = n_in_buffer;

  output->costs.reserve(n_buffer);
  output->costs = cost_data;
  output->configurations = config_data;
}

}  // namespace systems
}  // namespace dairlib