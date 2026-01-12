#include "mpm_model_reducer.h"

#include <algorithm>
#include <iostream>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_material_points.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MpmPointsToReducedModelPoints::MpmPointsToReducedModelPoints(
    Matrix3Xd support_directions)
    : n_support_directions_(support_directions.cols()),
      support_directions_(support_directions) {
  this->set_name("MpmPointsToReducedModelPoints");

  lcmt_material_points_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_material_points",
              drake::Value<dairlib::lcmt_material_points>{})
          .get_index();

  lcmt_timestamped_saved_traj_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_timestamped_saved_traj",
              dairlib::lcmt_timestamped_saved_traj(),
              &MpmPointsToReducedModelPoints::OutputReducedModelPointsLcm)
          .get_index();
}

void MpmPointsToReducedModelPoints::OutputReducedModelPointsLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  // Evaluate input port to get the MPM contents.
  const auto& material_points_lcmt =
      this->EvalInputValue<dairlib::lcmt_material_points>(
          context, lcmt_material_points_input_port_);

  // Check if it's too early to output or if there are no points to draw.
  Matrix3Xd reduced_points = Matrix3Xd::Zero(3, n_support_directions_);
  if ((material_points_lcmt->utime > 1e-3) &&
      (material_points_lcmt->num_points > 1)) {
    // Extract the MPM points.
    int n_points = material_points_lcmt->num_points;
    Matrix3Xd points = Matrix3Xd::Zero(3, n_points);
    for (int point_i = 0; point_i < n_points; point_i++) {
      std::vector<float> point = material_points_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        points(dim_i, point_i) = point[dim_i];
      }
    }

    // Compute the support point along each support direction.
    for (int dir_i = 0; dir_i < n_support_directions_; dir_i++) {
      VectorXd projections =
          support_directions_.col(dir_i).transpose() * points;
      Eigen::Index max_index;
      projections.maxCoeff(&max_index);
      reduced_points.col(dir_i) = points.col(max_index);
    }
  }

  // Use dummy timestamps (they have to be ascending).
  VectorXd timestamps = VectorXd::Zero(n_support_directions_);
  for (int dir_i = 0; dir_i < n_support_directions_; dir_i++) {
    timestamps(dir_i) = dir_i;
  }

  // Build the output type.
  LcmTrajectory::Trajectory reduced_points_traj;
  reduced_points_traj.traj_name = "reduced_model_points";
  reduced_points_traj.datatypes = std::vector<std::string>(3, "double");
  reduced_points_traj.datapoints = reduced_points;
  reduced_points_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory sample_traj({reduced_points_traj}, {"reduced_model_points"},
                            "reduced_model_points", "reduced_model_points",
                            false);
  output->saved_traj = sample_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
