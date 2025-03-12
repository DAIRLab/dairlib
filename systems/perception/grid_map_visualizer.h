#pragma once

#include "grid_map_core/grid_map_core.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/meshcat.h"

namespace dairlib {
namespace perception {

class GridMapVisualizer : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GridMapVisualizer);
  GridMapVisualizer(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                    double update_period_sec,
                    std::vector<std::string> layers = {});

  void DrawGridMap(const grid_map::GridMap& map,
                   const std::vector<std::string>& layers,
                   const std::string& prefix ="") const;

  void SetRgba(double r, double g, double b, double a) {
    rgba_ = drake::geometry::Rgba(r, g, b, a);
  }

 private:
  drake::systems::EventStatus UpdateVisualization(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  mutable std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::vector<std::string> layers_;

  drake::geometry::Rgba rgba_{0.1, 0.1, 0.9, 1.0};

};

} // dairlib
} // perception

