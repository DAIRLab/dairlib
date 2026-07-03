#include "elastoplastic_model_interpreter.h"

#include "dairlib/lcmt_elastoplastic_network.hpp"

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

ElastoPlasticModelInterpreter::ElastoPlasticModelInterpreter(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    int n_network_points, const std::string& node_model_file, VectorXd color)
    : meshcat_(meshcat), n_network_points_(n_network_points) {
  this->set_name("ElastoPlasticModelInterpreter");

  if (color.size() == 3) {
    connection_color_ =
        drake::geometry::Rgba(color(0), color(1), color(2), 1.0);
  } else {
    connection_color_ = drake::geometry::Rgba(0.8, 0.8, 0.2, 1.0);
  }

  lcmt_elastoplastic_network_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  // Weld "base_link" to world, since each node is a translation-only (x/y/z
  // prismatic) point relative to it rather than a floating body. Without the
  // weld, the node model's base_link picks up an extra implicit 7-DOF
  // floating joint, which desyncs the per-node position vector.
  node_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      node_model_file, n_network_points_,
      Eigen::VectorXd::Constant(n_network_points_, 1.0), "base_link",
      RigidTransformd(), meshcat, connection_path_, color, connection_path_);

  meshcat_->SetProperty(connection_path_, "visible", true, 0);

  last_update_time_index_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(
      &ElastoPlasticModelInterpreter::DrawNetwork);
}

drake::systems::EventStatus ElastoPlasticModelInterpreter::DrawNetwork(
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

  if (elastoplastic_model->num_points != n_network_points_) {
    throw std::runtime_error(
        "ElastoPlasticModelInterpreter: mismatch in number of network "
        "points.");
  }
  Matrix3Xd network_points = Matrix3Xd::Zero(3, n_network_points_);
  for (int point_i = 0; point_i < n_network_points_; point_i++) {
    for (int dim_i = 0; dim_i < 3; dim_i++) {
      network_points(dim_i, point_i) =
          elastoplastic_model->points[point_i][dim_i];
    }
  }
  node_visualizer_->DrawPoses(network_points);

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
