#include "deformable_drawer.h"

#include <cassert>

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace visualization {

DeformableDrawer::DeformableDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& path, const std::string& keypoint_path,
    const std::vector<int>& vertex_indices,
    const std::vector<std::pair<size_t, size_t>>& sphere_connections)
    : meshcat_(meshcat),
      path_(path),
      keypoint_path_(keypoint_path),
      vertex_indices_(vertex_indices),
      sphere_connections_(sphere_connections) {
  this->set_name("DeformableDrawer");
  drake::lcmt_viewer_link_data default_data;
  default_data.name = "default";
  default_data.robot_num = 0;
  default_data.num_geom = 0;
  default_data.geom.resize(0);
  deformable_geometry_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_viewer_link_data",
              drake::Value<drake::lcmt_viewer_link_data>{default_data})
          .get_index();
  DeclarePerStepDiscreteUpdateEvent(&DeformableDrawer::DrawDeformableGeometry);
}

drake::systems::EventStatus DeformableDrawer::DrawDeformableGeometry(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& deformable_geometries =
      this->EvalInputValue<drake::lcmt_viewer_link_data>(
          context, deformable_geometry_input_port_);
  if (deformable_geometries->num_geom == 0) {
    return drake::systems::EventStatus::Succeeded();
  }
  const std::string link_name = deformable_geometries->name;
  const int robot = deformable_geometries->robot_num;
  const std::string link_path =
      path_ + "/" + std::to_string(robot) + "/" + link_name;
  const auto& geom = deformable_geometries->geom[0];
  const std::string geom_name = geom.string_data;
  const std::string geom_path = link_path + "/" + geom_name;
  DRAKE_DEMAND(geom.type == drake::lcmt_viewer_geometry_data::MESH);

  Matrix3Xd vertices;
  Matrix3Xi faces;
  Rgba rgba;
  RigidTransformd pose;
  std::tie(vertices, faces, rgba, pose) = ConvertDeformableGeom(geom);
  meshcat_->SetTriangleMesh(geom_path, vertices, faces, rgba);
  meshcat_->SetTransform(link_path, pose);

  // Draw keypoints if provided
  if (!vertex_indices_.empty()) {
    const int num_verts = vertices.cols();

    // Place keypoints as children of link_path so they inherit the same
    // transform as the mesh Use geometry-local vertices (same frame as mesh
    // vertices) so they appear at the correct positions after the link_path
    // transform is applied
    const std::string keypoint_path = link_path + "/keypoints";

    for (size_t i = 0; i < vertex_indices_.size(); ++i) {
      const int vert_idx = vertex_indices_[i];
      if (vert_idx >= 0 && vert_idx < num_verts) {
        // Use geometry-local vertices (same frame as mesh vertices)
        // They will be transformed by the link_path transform, same as the mesh
        const Vector3d vertex_pos = vertices.col(vert_idx);
        const std::string sphere_path =
            keypoint_path + "/sphere_" + std::to_string(i);

        // Create and set sphere at vertex position
        Sphere sphere(0.005);
        meshcat_->SetObject(sphere_path, sphere, Rgba(0.0, 1.0, 0.0, 1.0));
        meshcat_->SetTransform(sphere_path, RigidTransformd(vertex_pos));
      }
    }

    // Draw purple lines connecting specified sphere pairs
    for (const auto& connection : sphere_connections_) {
      const size_t sphere_idx0 = connection.first;
      const size_t sphere_idx1 = connection.second;

      // Validate sphere indices
      if (sphere_idx0 >= vertex_indices_.size() ||
          sphere_idx1 >= vertex_indices_.size()) {
        continue;
      }

      const int vert_idx0 = vertex_indices_[sphere_idx0];
      const int vert_idx1 = vertex_indices_[sphere_idx1];

      if (vert_idx0 >= 0 && vert_idx0 < num_verts && vert_idx1 >= 0 &&
          vert_idx1 < num_verts) {
        // Use geometry-local vertices (same frame as mesh vertices)
        const Vector3d pos0 = vertices.col(vert_idx0);
        const Vector3d pos1 = vertices.col(vert_idx1);

        // SetLine expects a 3xN matrix where each column is a point
        Matrix3Xd line_points(3, 2);
        line_points.col(0) = pos0;
        line_points.col(1) = pos1;

        const std::string line_path = keypoint_path + "/line_" +
                                      std::to_string(sphere_idx0) + "_" +
                                      std::to_string(sphere_idx1);
        meshcat_->SetLine(line_path, line_points, 5.0, purple_color_);
      }
    }
  }

  return drake::systems::EventStatus::Succeeded();
}

std::tuple<Matrix3Xd, Matrix3Xi, Rgba, RigidTransformd>
DeformableDrawer::ConvertDeformableGeom(
    const drake::lcmt_viewer_geometry_data& geom) const {
  assert(geom.type == drake::lcmt_viewer_geometry_data::MESH);

  const int num_verts = static_cast<int>(geom.float_data[0]);
  const int num_faces = static_cast<int>(geom.float_data[1]);

  // Parse vertex and face data
  const int v_start = 2;
  const int f_start = v_start + 3 * num_verts;

  Matrix3Xd vertices(3, num_verts);
  for (int i = 0; i < num_verts; ++i) {
    vertices(0, i) = geom.float_data[v_start + 3 * i + 0];
    vertices(1, i) = geom.float_data[v_start + 3 * i + 1];
    vertices(2, i) = geom.float_data[v_start + 3 * i + 2];
  }

  Matrix3Xi faces(3, num_faces);
  for (int i = 0; i < num_faces; ++i) {
    faces(0, i) = static_cast<int>(geom.float_data[f_start + 3 * i + 0]);
    faces(1, i) = static_cast<int>(geom.float_data[f_start + 3 * i + 1]);
    faces(2, i) = static_cast<int>(geom.float_data[f_start + 3 * i + 2]);
  }

  // Color and pose
  Rgba rgba(geom.color[0], geom.color[1], geom.color[2], geom.color[3]);
  RigidTransformd pose(ToPose(geom.position, geom.quaternion));

  return std::make_tuple(vertices, faces, rgba, pose);
}

}  // namespace visualization
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
