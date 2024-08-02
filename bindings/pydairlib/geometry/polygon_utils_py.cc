#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "geometry/polygon_utils.h"
#include "geometry/polygon_height_map.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using geometry::PolygonHeightMap;
using geometry::ConvexPolygonSet;

PYBIND11_MODULE(polygon_utils, m) {
  m.doc() = "Binding geometry polygon simplification utils";

  m.def("ProcessTerrain2d", &geometry::ProcessTerrain2d, py::arg("terrain"),
        py::arg("concavity_threshold"))
   .def("Acd", &geometry::Acd, py::arg("verts"), py::arg("concavity_threshold"))
   .def("GetAcdComponents", &geometry::GetAcdComponents, py::arg("planar_region"));

  py::class_<PolygonHeightMap>(m, "PolygonHeightMap")
      .def(py::init<ConvexPolygonSet, double>())
      .def("CalcHeight", &PolygonHeightMap::CalcHeight);

}

}
}