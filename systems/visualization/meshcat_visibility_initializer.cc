#include "systems/visualization/meshcat_visibility_initializer.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace dairlib {
namespace systems {

using drake::geometry::Meshcat;
using drake::systems::Context;
using drake::systems::EventStatus;

MeshcatVisibilityInitializer::MeshcatVisibilityInitializer(
    std::shared_ptr<Meshcat> meshcat, std::vector<std::string> paths_to_hide,
    double check_period)
    : meshcat_(std::move(meshcat)), pending_paths_(std::move(paths_to_hide)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(check_period > 0);
  this->set_name("MeshcatVisibilityInitializer");

  this->DeclarePeriodicPublishEvent(
      check_period, 0.0, &MeshcatVisibilityInitializer::HidePathsThatExist);

  if (!pending_paths_.empty()) {
    std::string path_list;
    for (const std::string& path : pending_paths_) {
      path_list += (path_list.empty() ? "" : ", ") + path;
    }
    drake::log()->info(
        "Waiting to toggle off these meshcat paths, each once something draws "
        "it:  {}",
        path_list);
  }
}

EventStatus MeshcatVisibilityInitializer::HidePathsThatExist(
    const Context<double>&) const {
  if (pending_paths_.empty()) {
    return EventStatus::Succeeded();
  }

  std::vector<std::string> still_pending;
  for (const std::string& path : pending_paths_) {
    if (meshcat_->HasPath(path)) {
      meshcat_->SetProperty(path, "visible", false);
      drake::log()->info("Toggled off meshcat path '{}'.", path);
    } else {
      still_pending.push_back(path);
    }
  }
  pending_paths_ = std::move(still_pending);

  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
