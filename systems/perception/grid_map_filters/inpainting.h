#pragma once

#include <string>
#include <grid_map_core/grid_map_core.hpp>

namespace dairlib {
namespace perception {

void InpaintWithMinimumValues(grid_map::GridMap &map, const std::string &layerIn,
                              const std::string &layerOut);

}
}