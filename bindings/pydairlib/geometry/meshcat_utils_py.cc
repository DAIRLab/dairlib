#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "geometry/meshcat_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using geometry::MeshcatUtils;
using drake::geometry::Meshcat;

PYBIND11_MODULE(meshcat_utils, m) {
m.doc() = "Binding meshcat_utils";

using py_rvp = py::return_value_policy;

py::class_<MeshcatUtils>(m, "MeshcatUtils")
    .def(py::init<>())
    .def("PlotColoredSurface",
         &MeshcatUtils::PlotColoredSurface,
             py::arg("path"),py::arg("MeshcatObject"),py::arg("X"),py::arg("Y"),
             py::arg("Z"),py::arg("colors"),py::arg("wireframe"), py::arg("wireframe_line_width"));
}
}
}