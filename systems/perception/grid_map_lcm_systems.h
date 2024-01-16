#include "grid_map_core/grid_map_core.hpp"
#include "dairlib/lcmt_grid_map.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

lcmt_grid_map GridMapToLcm(
    const grid_map::GridMap& grid_map, const std::vector<std::string>& layers);

inline lcmt_grid_map GridMapToLcm(const grid_map::GridMap& grid_map) {
  return GridMapToLcm(grid_map, grid_map.getLayers());
}

grid_map::GridMap LcmToGridMap(const lcmt_grid_map& grid_map_lcm);

}
}