#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Toggles a set of meshcat paths off, each one as soon as it exists.
///
/// This is meant for paths that are worth keeping available but are in the way
/// by default:  they stay in the scene tree, so the Controls -> Scene checkbox
/// brings them back, but nothing is drawn until the user asks for it.
///
/// Why watch instead of hiding the paths up front:  `Meshcat::SetProperty`
/// *creates* the path it is given, so hiding a path before anything draws to it
/// adds an empty line item to the scene tree -- a checkbox that will never show
/// anything.  That is misleading for a mistyped path, and worse for a path
/// whose producer is switched off (with `visualize_sample_locations: false`,
/// nothing ever draws the sample pose traces), since the scene tree is how one
/// reads which drawers are on.  Waiting until the path exists means a path
/// nothing populates contributes nothing to the tree.
///
/// Each path is hidden exactly once and then forgotten, so re-checking its box
/// in the browser sticks;  a system that kept re-applying `visible=false` would
/// fight the user's toggle.
///
/// Two caveats worth knowing:
/// - A path that never appears is silently ignored -- that is the point, but it
///   means a typo is inert rather than loud.  The info log this system writes
///   when it hides a path (and the one naming what it is waiting on) is how to
///   tell a typo from a path that simply has not been drawn yet.
/// - A path whose *exact* node is later handed to `Meshcat::SetObject` comes
///   back visible, because meshcat.js replaces the node's object and rebuilds
///   its controls.  Hiding a folder (a path drawn *under*, which is the usual
///   case here) is unaffected.
class MeshcatVisibilityInitializer : public drake::systems::LeafSystem<double> {
 public:
  /// @param meshcat The meshcat instance whose scene tree is watched
  /// @param paths_to_hide Meshcat paths, spelled the way C++ spells them, i.e.
  ///   without the "Scene/drake/" prefix the browser shows
  /// @param check_period How often, in seconds, to look for the paths.  Note
  ///   that `Meshcat::HasPath` is a blocking round trip to meshcat's websocket
  ///   thread, so this should be no faster than the visualizer's frame rate;
  ///   once every path has been hidden the check costs nothing.
  MeshcatVisibilityInitializer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      std::vector<std::string> paths_to_hide, double check_period);

 private:
  drake::systems::EventStatus HidePathsThatExist(
      const drake::systems::Context<double>& context) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  /// The paths that have not shown up yet.  Mutable rather than Context state,
  /// matching how the Lcm*Drawer systems here and Drake's own MeshcatVisualizer
  /// hold what they have already sent to meshcat.
  mutable std::vector<std::string> pending_paths_;
};

}  // namespace systems
}  // namespace dairlib
