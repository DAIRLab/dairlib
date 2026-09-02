#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "multibody/multipose_visualizer.h"

#include "drake/geometry/meshcat.h"

namespace dairlib {
namespace systems {

/// Draws static (world-welded) models into an existing Meshcat instance.
///
/// This complements the Lcm*Drawer systems in lcm_visualization_systems.h:
/// nothing here participates in a Diagram.  Each model is drawn once, at its
/// default pose, under a caller-chosen meshcat path so that it can be toggled
/// on its own.
class StaticModelDrawer {
 public:
  /// Alpha used for a color given as [r, g, b].
  static constexpr double kDefaultAlpha = 0.25;

  explicit StaticModelDrawer(std::shared_ptr<drake::geometry::Meshcat> meshcat);

  /// Draws @p model_file in its default configuration.
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param weld_frame_to_world Frame in the model welded to the world origin
  /// @param meshcat_path The meshcat subtree to draw the model under; bodies
  ///   land at `{meshcat_path}/{model name}/{link name}`
  /// @param color Empty to use the colors defined in the model file, [r, g, b]
  ///   (with an alpha of kDefaultAlpha), or [r, g, b, a].  Note that bodies
  ///   which do define a material have their alpha *scaled* rather than set,
  ///   matching how MultiposeVisualizer treats alpha elsewhere; bodies with no
  ///   material take the color exactly.
  void AddModel(const std::string& model_file,
                const std::string& weld_frame_to_world,
                const std::string& meshcat_path,
                const Eigen::VectorXd& color = Eigen::VectorXd());

 private:
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::vector<std::unique_ptr<multibody::MultiposeVisualizer>> visualizers_;
};

}  // namespace systems
}  // namespace dairlib
