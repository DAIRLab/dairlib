#pragma once

#include <utility>
#include <vector>

#include <drake/common/drake_assert.h>
#include <drake/common/eigen_types.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/rgba.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>

namespace dairlib {
namespace magna {

using drake::geometry::Rgba;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// Receives deformable geometry via lcmt messages with type
// lcmt_viewer_link_data which are emitted by Drake visualizer and draws it
// through meshcat. Optionally draws keypoints (spheres) at specified vertex
// indices with purple lines connecting specified pairs of spheres.
// sphere_connections specifies which sphere pairs to connect (indices into
// vertex_indices vector, e.g., {0,1} connects the first two spheres).
class DeformableDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit DeformableDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                            const std::string& path,
                            const std::string& keypoint_path,
                            const std::vector<int>& vertex_indices = {},
                            const std::vector<std::pair<size_t, size_t>>&
                                sphere_connections = {});

 private:
  drake::systems::EventStatus DrawDeformableGeometry(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::tuple<Matrix3Xd, Matrix3Xi, Rgba, RigidTransformd> ConvertDeformableGeom(
      const drake::lcmt_viewer_geometry_data& geom) const;

  static RigidTransformd ToPose(const std::array<float, 3>& position,
                                const std::array<float, 4>& quaternion) {
    const Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2],
                               quaternion[3]);
    const Eigen::Vector3d p(position[0], position[1], position[2]);
    return RigidTransformd(q, p);
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::systems::InputPortIndex deformable_geometry_input_port_;
  std::string path_;
  std::string keypoint_path_;
  std::vector<int> vertex_indices_;
  std::vector<std::pair<size_t, size_t>> sphere_connections_;
  const Rgba purple_color_ = Rgba(0.5, 0.0, 0.5, 1.0);  // Purple color
};

}  // namespace magna
}  // namespace dairlib
