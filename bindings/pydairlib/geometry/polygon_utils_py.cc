#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "geometry/polygon_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {


PYBIND11_MODULE(polygon_utils, m) {
  m.doc() = "Binding geometry polygon simplification utils";

  m.def("ProcessTerrain2d", &geometry::ProcessTerrain2d, py::arg("terrain"))
   .def("Acd", &geometry::Acd, py::arg("verts"), py::arg("concavity_threshold"))
   .def("GetAcdComponents", &geometry::GetAcdComponents, py::arg("planar_region"));


}

}
}