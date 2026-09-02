#include "systems/visualization/static_visualization_systems.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/rgba.h"

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::Rgba;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

StaticModelDrawer::StaticModelDrawer(std::shared_ptr<Meshcat> meshcat)
    : meshcat_(std::move(meshcat)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
}

void StaticModelDrawer::AddModel(const std::string& model_file,
                                 const std::string& weld_frame_to_world,
                                 const std::string& meshcat_path,
                                 const VectorXd& color) {
  DRAKE_DEMAND(color.size() == 0 || color.size() == 3 || color.size() == 4);

  MeshcatVisualizerParams params;
  params.prefix = meshcat_path;

  // Bodies that define a material are recolored by MultiposeVisualizer via
  // `rgb` and `alpha_scale`; bodies that define none fall through to
  // MeshcatVisualizer's default_color.  Set both so the model looks the same
  // either way.
  VectorXd rgb;
  double alpha = 1.0;
  if (color.size() > 0) {
    rgb = color.head(3);
    alpha = color.size() == 4 ? color(3) : kDefaultAlpha;
    params.default_color = Rgba(rgb(0), rgb(1), rgb(2), alpha);
  } else {
    params.default_color = Rgba(0.9, 0.2, 0.2, kDefaultAlpha);
  }

  visualizers_.push_back(std::make_unique<multibody::MultiposeVisualizer>(
      model_file, /* num_poses = */ 1, VectorXd::Constant(1, alpha),
      weld_frame_to_world, meshcat_, /* pose_trace_name = */ "", rgb,
      std::move(params)));

  // The model is welded to the world, so it has no positions to set; this
  // publish is what actually sends the geometry to meshcat.
  visualizers_.back()->DrawPoses(MatrixXd::Zero(0, 1));
}

}  // namespace systems
}  // namespace dairlib
