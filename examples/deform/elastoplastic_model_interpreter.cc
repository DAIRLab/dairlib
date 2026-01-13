#include "elastoplastic_model_interpreter.h"

#include <algorithm>
#include <iostream>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_elastoplastic_network.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace systems {

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

/// TODO a better implementation would directly draw the network points instead
/// of translating it into a saved traj and using LcmPoseDrawer.
ElastoPlasticModelInterpreter::ElastoPlasticModelInterpreter(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    int n_network_points)
    : meshcat_(meshcat), n_network_points_(n_network_points) {
  this->set_name("ElastoPlasticModelInterpreter");

  lcmt_elastoplastic_network_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  points_lcmt_timestamped_saved_traj_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_timestamped_saved_traj",
              dairlib::lcmt_timestamped_saved_traj(),
              &ElastoPlasticModelInterpreter::OutputReducedModelPointsLcm)
          .get_index();

  meshcat_->SetProperty(connection_path_, "visible", true, 0);

  last_update_time_index_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(
      &ElastoPlasticModelInterpreter::DrawConnections);
}

void ElastoPlasticModelInterpreter::OutputReducedModelPointsLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  // Evaluate input port to get the elastoplastic network contents.
  const auto& elastoplastic_network_lcmt =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          context, lcmt_elastoplastic_network_input_port_);

  // Check if it's too early to output or if there are no points to draw.
  Matrix3Xd network_points = Matrix3Xd::Zero(3, n_network_points_);
  if ((elastoplastic_network_lcmt->utime > 1e-3) &&
      (elastoplastic_network_lcmt->num_points > 1)) {
    // Extract the MPM points.
    if (n_network_points_ != elastoplastic_network_lcmt->num_points) {
      throw std::runtime_error(
          "ElastoPlasticModelInterpreter: mismatch in number of network "
          "points.");
    }
    for (int point_i = 0; point_i < n_network_points_; point_i++) {
      std::vector<float> point = elastoplastic_network_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        network_points(dim_i, point_i) = point[dim_i];
      }
    }
  }

  // Use dummy timestamps (they have to be ascending).
  VectorXd timestamps = VectorXd::Zero(n_network_points_);
  for (int point_i = 0; point_i < n_network_points_; point_i++) {
    timestamps(point_i) = point_i;
  }

  // Build the output type.
  LcmTrajectory::Trajectory network_points_traj;
  network_points_traj.traj_name = "network_points";
  network_points_traj.datatypes = std::vector<std::string>(3, "double");
  network_points_traj.datapoints = network_points;
  network_points_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory network_point_traj({network_points_traj}, {"network_points"},
                                   "network_points", "network_points", false);
  output->saved_traj = network_point_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

drake::systems::EventStatus ElastoPlasticModelInterpreter::DrawConnections(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
              context, lcmt_elastoplastic_network_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& elastoplastic_model =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          context, lcmt_elastoplastic_network_input_port_);

  // Don't needlessly update
  if (discrete_state->get_value(last_update_time_index_)[0] ==
      elastoplastic_model->utime * 1e-6) {
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      elastoplastic_model->utime * 1e-6;

  for (int i = 0; i < elastoplastic_model->num_connections; ++i) {
    int vertex_i = elastoplastic_model->connections[i][0];
    int vertex_j = elastoplastic_model->connections[i][1];
    const VectorXd point_1 = Vector3d(elastoplastic_model->points[vertex_i][0],
                                      elastoplastic_model->points[vertex_i][1],
                                      elastoplastic_model->points[vertex_i][2]);
    const VectorXd point_2 = Vector3d(elastoplastic_model->points[vertex_j][0],
                                      elastoplastic_model->points[vertex_j][1],
                                      elastoplastic_model->points[vertex_j][2]);
    const VectorXd vector_1_to_2 = point_2 - point_1;
    auto distance_norm = vector_1_to_2.norm();
    const std::string& conn_path_root = connection_path_ + "/vertex_" +
                                        std::to_string(vertex_i) + "_to_" +
                                        std::to_string(vertex_j) + "/";
    if (distance_norm >= 1e-3) {
      if (!meshcat_->HasPath(conn_path_root + "arrow/")) {
        meshcat_->SetObject(conn_path_root + "arrow/cylinder", cylinder_,
                            connection_color_);
      }
      meshcat_->SetTransform(conn_path_root, RigidTransformd(point_1),
                             context.get_time());
      // Transform and stretch the cylinder (in z) to match the length of the
      // connection.
      std::string conn_arrow_path = conn_path_root + "arrow";
      meshcat_->SetTransform(
          conn_arrow_path,
          RigidTransformd(RotationMatrixd::MakeFromOneVector(vector_1_to_2, 2)),
          context.get_time());
      meshcat_->SetProperty(conn_arrow_path + "/cylinder", "position",
                            {0, 0, 0.5 * distance_norm}, context.get_time());
      // Note: Meshcat does not fully support non-uniform scaling (see
      // #18095). We get away with it here since there is no rotation on this
      // frame and no children in the kinematic tree.
      meshcat_->SetProperty(conn_arrow_path + "/cylinder", "scale",
                            {1, 1, distance_norm}, context.get_time());
      meshcat_->SetProperty(conn_path_root, "visible", true,
                            context.get_time());
    } else {
      meshcat_->SetProperty(conn_path_root, "visible", false,
                            context.get_time());
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
