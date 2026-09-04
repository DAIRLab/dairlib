#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <drake/perception/point_cloud.h>

#include "systems/senders/cost_colormap.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

using drake::perception::PointCloud;

namespace systems {

class PointCloudFromSampleBuffer : public drake::systems::LeafSystem<double> {
  // This system reads the sample_buffer over lcm and outputs a Drake PointCloud
  // for visualization.
 public:
  PointCloudFromSampleBuffer();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_lcmt_sample_buffer()
      const {
    return this->get_input_port(lcmt_sample_buffer_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_new_sample_costs()
      const {
    return this->get_input_port(new_sample_costs_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>&
  get_output_port_sample_buffer_point_cloud() const {
    return this->get_output_port(point_cloud_output_port_);
  }

 private:
  void OutputSampleBufferAsPointCloud(
      const drake::systems::Context<double>& context,
      PointCloud* sample_buffer_point_cloud) const;

  drake::systems::InputPortIndex new_sample_costs_input_port_;
  drake::systems::InputPortIndex lcmt_sample_buffer_input_port_;
  drake::systems::OutputPortIndex point_cloud_output_port_;

  // The RdYlGn.reversed() colormap this uses lives in
  // systems/senders/cost_colormap.h.
  Eigen::VectorXf color_floats_;
  Eigen::MatrixXi RGBs_;

  const int n_colors_ = kNumColormapEntries;
};

}  // namespace systems
}  // namespace dairlib
