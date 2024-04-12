#include "systems/perception/grid_map_filters/inpainting.h"

namespace dairlib {
namespace perception {

void InpaintWithMinimumValues(grid_map::GridMap &map,
                              const std::string &layerIn,
                              const std::string &layerOut) {
/*
 Implementation copied from ETH planar segmentation library:
 https://github.com/leggedrobotics/elevation_mapping_cupy/blob/8093dfb6090f449bafc878198fafb025870532b6/plane_segmentation/grid_map_filters_rsl/src/inpainting.cpp#L25

 License:

MIT License

 Copyright (c) 2022 ETH Zurich, Takahiro Miki

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in, and out maps, initialize with copy
  const grid_map::Matrix &H_in = map.get(layerIn);
  grid_map::Matrix &H_out = map.get(layerOut);
  H_out = H_in;

  // Some constant
  const int numCols = H_in.cols();
  const int maxColId = numCols - 1;
  const int numRows = H_in.rows();
  const int maxRowId = numRows - 1;

  // Common operation of updating the minimum and keeping track if the minimum was updated.
  auto compareAndStoreMin =
      [](float newValue, float &currentMin, bool &changedValue) {
        if (!std::isnan(newValue)) {
          if (newValue < currentMin || std::isnan(currentMin)) {
            currentMin = newValue;
            changedValue = true;
          }
        }
      };

  /*
   * Fill each cell that needs inpainting with the min of its neighbours until the map doesn't change anymore.
   * This way each inpainted area gets the minimum value along its contour.
   *
   * We will be reading and writing to H_out during iteration. However, the aliasing does not break the correctness of the algorithm.
   */
  bool hasAtLeastOneValue = true;
  bool changedValue = true;
  while (changedValue && hasAtLeastOneValue) {
    hasAtLeastOneValue = false;
    changedValue = false;
    for (int colId = 0; colId < numCols; ++colId) {
      for (int rowId = 0; rowId < numRows; ++rowId) {
        if (std::isnan(H_in(rowId, colId))) {
          auto &middleValue = H_out(rowId, colId);

          // left
          if (colId > 0) {
            const auto leftValue = H_out(rowId, colId - 1);
            compareAndStoreMin(leftValue, middleValue, changedValue);
          }
          // right
          if (colId < maxColId) {
            const auto rightValue = H_out(rowId, colId + 1);
            compareAndStoreMin(rightValue, middleValue, changedValue);
          }
          // top
          if (rowId > 0) {
            const auto topValue = H_out(rowId - 1, colId);
            compareAndStoreMin(topValue, middleValue, changedValue);
          }
          // bottom
          if (rowId < maxRowId) {
            const auto bottomValue = H_out(rowId + 1, colId);
            compareAndStoreMin(bottomValue, middleValue, changedValue);
          }
        } else {
          hasAtLeastOneValue = true;
        }
      }
    }
  }
}

} // perception
} // dairlib