#include "systems/perception/grid_map_filters/inpainting.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(grid_map_filters, m) {
  m.def("InpaintWithMinimumValues", &dairlib::perception::InpaintWithMinimumValues);
}