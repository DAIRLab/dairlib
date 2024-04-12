#pragma once
#include "grid_map_core/grid_map_core.hpp"
#include "dairlib/lcmt_grid_map.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

lcmt_grid_map GridMapToLcm(
    const grid_map::GridMap& grid_map, const std::vector<std::string>& layers);

lcmt_grid_map GridMapToLcm(const grid_map::GridMap& grid_map);

grid_map::GridMap LcmToGridMap(const lcmt_grid_map& grid_map_lcm);

class GridMapSender : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GridMapSender);
  GridMapSender();

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  lcmt_grid_map* out) const;

};

class GridMapReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GridMapReceiver);
  GridMapReceiver();

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  grid_map::GridMap* out) const;

};

}
}