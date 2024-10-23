#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "grid_map_core/grid_map_core.hpp"

namespace dairlib {
namespace perception {

class TerrainSegmentationMonitor : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TerrainSegmentationMonitor);
  static constexpr size_t kMaxBufferLen = 15;

  TerrainSegmentationMonitor(double update_period, size_t lookback_size);



  bool NeedsIoUReset(const drake::systems::Context<double>& context) const;
  bool NeedsMinValidAreaReset(const drake::systems::Context<double>& context) const;

  grid_map::GridMap GetMapForReInitialization(
      const drake::systems::Context<double>& context) const;

 private:

  drake::systems::EventStatus Initialize(
      const drake::systems::Context<double>&,
      drake::systems::State<double>* state) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const final;


  drake::systems::InputPortIndex input_port_grid_map_;
  drake::systems::AbstractStateIndex map_history_index_;
  double lookback_;
  double iou_threshold_ = 0.7;
  double area_threshold = 0.5;
};

}
}