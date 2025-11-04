#include "deformable_drawer.h"

#include <iostream>

namespace dairlib {
namespace magna {

DeformableDrawer::DeformableDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& path)
    : meshcat_(meshcat), path_(path) {
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
  return drake::systems::EventStatus::Succeeded();
}

std::tuple<Matrix3Xd, Matrix3Xi, Rgba, RigidTransformd>
DeformableDrawer::ConvertDeformableGeom(
    const drake::lcmt_viewer_geometry_data& geom) const {
  assert(geom.type == lcmt_viewer_geometry_data::MESH);

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

}  // namespace magna
}  // namespace dairlib
