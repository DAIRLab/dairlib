#pragma once

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCWalkingDebugVisualizer : public drake::systems::LeafSystem<double> {
 public:
  explicit IDMPCWalkingDebugVisualizer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat);

 private:

  void UpdateMeshcat(const drake::systems::Context<double> &context) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};

}