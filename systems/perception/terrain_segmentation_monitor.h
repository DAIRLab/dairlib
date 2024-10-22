#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "grid_map_core/grid_map_core.hpp"

namespace dairlib {
namespace perception {

class TerrainSegmentationMonitor : public drake::systems::LeafSystem<double> {
 public:

  static constexpr size_t kMaxBufferLen = 15;
  TerrainSegmentationMonitor(double update_period, size_t num_buffer_periods);

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TerrainSegmentationMonitor);

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const final;

  bool NeedsIoUReset(const drake::systems::Context<double>& context);
  bool NeedsMinValidAreaReset(const drake::systems::Context<double>& context);

  grid_map::GridMap GetMapForReInitialization() const;

 private:

  drake::systems::AbstractStateIndex map_history_index_;
  double lookback_;

};

}
}