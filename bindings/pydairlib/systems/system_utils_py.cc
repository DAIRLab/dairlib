#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/system_utils.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;

namespace dairlib::pydairlib {
  PYBIND11_MODULE(system_utils, m) {
  m.doc() = "Binding system utilities";
  m.def("DrawAndSaveDiagramGraph", &dairlib::DrawAndSaveDiagramGraph, py::arg("diagram"), py::arg("path"));
}
}