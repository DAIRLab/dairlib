#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/meshcat.h"

namespace dairlib {
namespace perceptive_locomotion {

class MeshcatFootholdVisualizer : public drake::systems::LeafSystem<double> {
 public:
  MeshcatFootholdVisualizer(std::shared_ptr<drake::geometry::Meshcat> meshcat);
 private:
  static std::string make_path(int i) {
    return "/foothold_meshes/" + std::to_string(i);
  }

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  drake::systems::InputPortIndex mpc_debug_input_port_;
  drake::systems::DiscreteStateIndex n_footholds_idx_;
  mutable std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};

}
}

