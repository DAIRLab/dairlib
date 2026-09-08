#include "sample_buffer_sender.h"

#include <algorithm>
#include <iostream>
#include <limits>

#include "common/eigen_utils.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {
// Number of jam columns the jam data input port carries, matching
// kNumJamColumns of JamBufferColumn in
// systems/controllers/sampling_based_c3_controller.h.  Duplicated rather than
// included because systems/senders does not depend on the controller.
constexpr int kNumJamColumnsExpected = 3;
constexpr int kJamLabelColumn = 0;
constexpr int kJamTravelColumn = 1;
constexpr int kPlanIsRealColumn = 2;
constexpr float kUnlabelled = std::numeric_limits<float>::quiet_NaN();
}  // namespace

SampleBufferSender::SampleBufferSender(int buffer_size, int n_config,
                                       std::string name)
    : buffer_size_(buffer_size), n_config_(n_config) {
  this->set_name(name);

  MatrixXd sample_buffer = MatrixXd::Zero(buffer_size_, n_config_);
  VectorXd cost_buffer = VectorXd::Zero(buffer_size_);
  samples_port_ =
      this->DeclareAbstractInputPort("sample_buffer_configurations",
                                     drake::Value<MatrixXd>{sample_buffer})
          .get_index();
  sample_costs_port_ =
      this->DeclareAbstractInputPort("sample_buffer_costs",
                                     drake::Value<VectorXd>{cost_buffer})
          .get_index();
  MatrixXd jam_buffer = MatrixXd::Constant(buffer_size_, kNumJamColumnsExpected,
                                           kUnlabelled);
  jam_data_port_ =
      this->DeclareAbstractInputPort("sample_buffer_jam_data",
                                     drake::Value<MatrixXd>{jam_buffer})
          .get_index();

  lcm_sample_buffer_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_sample_buffer", dairlib::lcmt_sample_buffer(),
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

  DRAKE_ASSERT(buffer_configurations->rows() == buffer_size_);
  DRAKE_ASSERT(buffer_configurations->cols() == n_config_);
  DRAKE_ASSERT(buffer_costs->size() == buffer_size_);

  // Count the number of active samples in the buffer.
  int n_in_buffer = std::count_if(buffer_costs->begin(), buffer_costs->end(),
                                  [](double cost) { return cost >= 0; });

  // Convert the Eigen matrices to std::vectors.
  std::vector<float> cost_data(buffer_costs->data(),
                               buffer_costs->data() + buffer_size_);
  std::vector<std::vector<float>> config_data(buffer_size_,
                                              std::vector<float>(n_config_, 0));
  for (int i = 0; i < buffer_size_; i++) {
    for (int j = 0; j < n_config_; j++) {
      config_data[i][j] = buffer_configurations->row(i)(j);
    }
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->buffer_length = buffer_size_;
  output->num_configurations = n_config_;
  output->num_in_buffer = n_in_buffer;

  output->costs.reserve(buffer_size_);
  output->costs = cost_data;
  output->configurations = config_data;

  // The jam data is optional:  a controller with no jam labeller configured
  // declares no jam output port to connect here, and publishes all NaN rather
  // than claiming a verdict it never computed.
  output->jam_labels = std::vector<float>(buffer_size_, kUnlabelled);
  output->jam_travel = std::vector<float>(buffer_size_, kUnlabelled);
  output->plan_is_real = std::vector<float>(buffer_size_, kUnlabelled);
  if (get_input_port_jam_data().HasValue(context)) {
    const auto& buffer_jam_data =
        this->EvalInputValue<MatrixXd>(context, jam_data_port_);
    DRAKE_ASSERT(buffer_jam_data->rows() == buffer_size_);
    DRAKE_ASSERT(buffer_jam_data->cols() == kNumJamColumnsExpected);
    for (int i = 0; i < buffer_size_; i++) {
      output->jam_labels[i] = (*buffer_jam_data)(i, kJamLabelColumn);
      output->jam_travel[i] = (*buffer_jam_data)(i, kJamTravelColumn);
      output->plan_is_real[i] = (*buffer_jam_data)(i, kPlanIsRealColumn);
    }
  }
}

}  // namespace systems
}  // namespace dairlib
