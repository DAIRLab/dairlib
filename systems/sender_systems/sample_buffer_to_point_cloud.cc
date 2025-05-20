#include "dairlib/lcmt_sample_buffer.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/perception/point_cloud.h"
#include "drake/perception/point_cloud_flags.h"
#include "drake/systems/framework/leaf_system.h"

#include "systems/sender_systems/sample_buffer_to_point_cloud.h"
#include <iostream>

namespace dairlib {

using drake::perception::PointCloud;
using drake::systems::Context;

namespace systems {

PointCloudFromSampleBuffer::PointCloudFromSampleBuffer() {
  this->set_name("PointCloudFromSampleBuffer");

  lcmt_sample_buffer_input_port_ = this->DeclareAbstractInputPort(
    "lcmt_sample_buffer", 
    drake::Value<dairlib::lcmt_sample_buffer>{}).get_index();

  new_sample_costs_input_port_ = this->DeclareAbstractInputPort(
    "new_sample_costs", 
    drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();

  point_cloud_output_port_ = this->DeclareAbstractOutputPort(
    "sample_buffer_point_cloud", PointCloud(),
    &PointCloudFromSampleBuffer::OutputSampleBufferAsPointCloud).get_index();

  color_floats_ = colorFloatMap();
  RGBs_ = RGBMap();
}

void PointCloudFromSampleBuffer::OutputSampleBufferAsPointCloud(
  const drake::systems::Context<double>& context,
  PointCloud* sample_buffer_point_cloud) const {
  
  if (!sample_buffer_point_cloud->HasExactFields(
    drake::perception::pc_flags::kXYZs | drake::perception::pc_flags::kRGBs)) {
      sample_buffer_point_cloud->SetFields(drake::perception::pc_flags::kXYZs |
                                           drake::perception::pc_flags::kRGBs);
  }

  // Evaluate input ports to get the sample buffer contents and current location
  // C3 cost.
  const auto& sample_buffer_lcmt =
    this->EvalInputValue<dairlib::lcmt_sample_buffer>(
      context, lcmt_sample_buffer_input_port_);
  const auto& new_sample_costs_traj_lcmt =
    this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, new_sample_costs_input_port_);

  Eigen::Matrix3Xf ee_samples;
  Eigen::Matrix3Xi rgbs;

  // Check if it's too early to output or if there are no samples in the buffer.
  if ((sample_buffer_lcmt->utime < 1e-3) ||
      (sample_buffer_lcmt->num_in_buffer < 1)) {
    ee_samples = Eigen::Matrix3Xf::Zero(3, 1);
    rgbs = Eigen::Matrix3Xi::Zero(3, 1);
  }
  else {
    // Extract the end effector locations out of the configurations and the 
    // samples' associated costs.
    int n_in_buffer = sample_buffer_lcmt->num_in_buffer;
    ee_samples = Eigen::Matrix3Xf::Zero(3, n_in_buffer);
    Eigen::VectorXf costs = Eigen::VectorXf::Zero(n_in_buffer);

    for (int sample_i = 0; sample_i < n_in_buffer; sample_i++) {
      costs[sample_i] = sample_buffer_lcmt->costs[sample_i];
      std::vector<float> configuration_i =
        sample_buffer_lcmt->configurations[sample_i];
      for (int ee_i = 0; ee_i < 3; ee_i ++) {
        ee_samples(ee_i, sample_i) = configuration_i[ee_i];
      }
    }

    // Normalize the costs to be between 0 and 1.
    if (costs.maxCoeff() == costs.minCoeff()) {
      rgbs = Eigen::Matrix3Xi::Zero(3, n_in_buffer);
    }
    else {
      Eigen::VectorXf normalized_costs = (costs.array() - costs.minCoeff()) /
        (costs.maxCoeff() - costs.minCoeff());

      // Get the current sample cost.
      auto lcm_traj = LcmTrajectory(new_sample_costs_traj_lcmt->saved_traj);
      auto lcm_sample_costs_traj = lcm_traj.GetTrajectory("sample_costs");
      Eigen::MatrixXd sample_costs = lcm_sample_costs_traj.datapoints.row(0);
      double current_cost = sample_costs(0);

      double normalized_current_cost = (current_cost - costs.minCoeff()) /
        (costs.maxCoeff() - costs.minCoeff());

      // Make the current cost the centered color.
      Eigen::VectorXf scaled_costs = normalized_costs.array() -
        normalized_current_cost + 0.5;

      // Apply the color map.
      Eigen::MatrixXf differences =
        (scaled_costs.replicate(1, n_colors_) -
        color_floats_.transpose().replicate(n_in_buffer, 1)).cwiseAbs();
      Eigen::VectorXi closest_color_indices(n_in_buffer);
      for (int i = 0; i < n_in_buffer; i++) {
        int idx;
        differences.row(i).minCoeff(&idx);
        closest_color_indices[i] = idx;
      }
      rgbs = Eigen::Matrix3Xi::Zero(3, n_in_buffer);
      for (int i = 0; i < n_in_buffer; i++) {
        rgbs.col(i) = RGBs_.row(closest_color_indices(i));
      }
    }
  }

  // Output as a point cloud.  TODO eventually change colors to reflect cost.
  sample_buffer_point_cloud->resize(ee_samples.cols());
  sample_buffer_point_cloud->mutable_xyzs() = ee_samples;
  sample_buffer_point_cloud->mutable_rgbs() = rgbs.cast<uint8_t>();
}

}  // namespace systems
}  // namespace dairlib