#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "geometry/meshcat_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using geometry::MeshcatUtils;

PYBIND11_MODULE(meshcat_utils, m) {
    m.doc() = "Binding meshcat_utils";
    m.def("PlotColoredSurface",
         &MeshcatUtils::PlotColoredSurface,
         py::arg("path"),py::arg("MeshcatObject"),
         py::arg("X"), py::arg("Y"), py::arg("Z"),
         py::arg("R"), py::arg("G"), py::arg("B"),
         py::arg("wireframe") = false,
         py::arg("wireframe_line_width") = 1.0);
}
}
}