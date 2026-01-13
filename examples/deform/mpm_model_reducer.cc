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
using Eigen::Matrix2Xi;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MpmPointsToReducedModel::MpmPointsToReducedModel(Matrix3Xd support_directions,
                                                 Matrix2Xi connections)
    : n_support_directions_(support_directions.cols()),
      support_directions_(support_directions),
      n_connections_(connections.cols()),
      connections_(connections) {
  this->set_name("MpmPointsToReducedModel");

  lcmt_material_points_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_material_points",
              drake::Value<dairlib::lcmt_material_points>{})
          .get_index();

  lcmt_elastoplastic_network_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_elastoplastic_network",
              dairlib::lcmt_elastoplastic_network(),
              &MpmPointsToReducedModel::OutputReducedModelNetworkLcm)
          .get_index();

  // Pre-compute connections data for LCM message.
  connections_data_ =
      std::vector<std::vector<int>>(n_connections_, std::vector<int>(2, 0));
  for (int i = 0; i < n_connections_; i++) {
    for (int j = 0; j < 2; j++) {
      connections_data_[i][j] = connections_.col(i)(j);
    }
  }
}

void MpmPointsToReducedModel::OutputReducedModelNetworkLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_elastoplastic_network* output) const {
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

  // Convert the Eigen matrices to std::vectors.
  std::vector<std::vector<float>> points_data(n_support_directions_,
                                              std::vector<float>(3, 0));
  for (int i = 0; i < n_support_directions_; i++) {
    for (int j = 0; j < 3; j++) {
      points_data[i][j] = reduced_points.col(i)(j);
    }
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->num_points = n_support_directions_;
  output->num_connections = n_connections_;
  output->points = points_data;
  output->connections = connections_data_;
}

}  // namespace systems
}  // namespace dairlib
