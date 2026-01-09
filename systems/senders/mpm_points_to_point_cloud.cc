#include "systems/senders/mpm_points_to_point_cloud.h"

#include "dairlib/lcmt_material_points.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/perception/point_cloud.h"
#include "drake/perception/point_cloud_flags.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

using drake::perception::PointCloud;
using drake::systems::Context;

namespace systems {

PointCloudFromMpmPoints::PointCloudFromMpmPoints() {
  this->set_name("PointCloudFromMpmPoints");

  lcmt_material_points_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_material_points",
              drake::Value<dairlib::lcmt_material_points>{})
          .get_index();

  point_cloud_output_port_ =
      this->DeclareAbstractOutputPort(
              "mpm_point_cloud", PointCloud(),
              &PointCloudFromMpmPoints::OutputMaterialPointsAsPointCloud)
          .get_index();

  color_floats_ = colorFloatMap();
  RGBs_ = RGBMap();
}

void PointCloudFromMpmPoints::OutputMaterialPointsAsPointCloud(
    const drake::systems::Context<double>& context,
    PointCloud* mpm_point_cloud) const {
  if (!mpm_point_cloud->HasExactFields(drake::perception::pc_flags::kXYZs |
                                       drake::perception::pc_flags::kRGBs)) {
    mpm_point_cloud->SetFields(drake::perception::pc_flags::kXYZs |
                               drake::perception::pc_flags::kRGBs);
  }

  // Evaluate input port to get the MPM contents.
  const auto& material_points_lcmt =
      this->EvalInputValue<dairlib::lcmt_material_points>(
          context, lcmt_material_points_input_port_);

  Eigen::Matrix3Xf points;
  Eigen::Matrix3Xi rgbs;

  // Check if it's too early to output or if there are no points to draw.
  if ((material_points_lcmt->utime < 1e-3) ||
      (material_points_lcmt->num_points < 1)) {
    points = Eigen::Matrix3Xf::Zero(3, 1);
    rgbs = Eigen::Matrix3Xi::Zero(3, 1);
  } else {
    // Extract the MPM points.
    int n_points = material_points_lcmt->num_points;
    points = Eigen::Matrix3Xf::Zero(3, n_points);
    Eigen::VectorXf costs = Eigen::VectorXf::Zero(n_points);

    for (int point_i = 0; point_i < n_points; point_i++) {
      std::vector<float> point = material_points_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        points(dim_i, point_i) = point[dim_i];
      }
    }

    // For visual purposes, color the points as a gradient, using the whole
    // color map.
    Eigen::VectorXf point_scale = Eigen::VectorXf::LinSpaced(n_points, 0, 1);
    Eigen::MatrixXf differences =
        (point_scale.replicate(1, n_colors_) -
         color_floats_.transpose().replicate(n_points, 1))
            .cwiseAbs();
    Eigen::VectorXi closest_color_indices(n_points);
    for (int i = 0; i < n_points; i++) {
      int idx;
      differences.row(i).minCoeff(&idx);
      closest_color_indices[i] = idx;
    }
    rgbs = Eigen::Matrix3Xi::Zero(3, n_points);
    for (int i = 0; i < n_points; i++) {
      rgbs.col(i) = RGBs_.row(closest_color_indices(i));
    }
  }

  // Output as a point cloud where the colors reflect costs.
  mpm_point_cloud->resize(points.cols());
  mpm_point_cloud->mutable_xyzs() = points;
  mpm_point_cloud->mutable_rgbs() = rgbs.cast<uint8_t>();
}

}  // namespace systems
}  // namespace dairlib
