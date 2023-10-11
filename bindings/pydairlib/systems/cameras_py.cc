#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/perception/camera_utils.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;

namespace dairlib::pydairlib {
PYBIND11_MODULE(cameras, m) {
  m.doc() = "Binding camera utilities";
  m.def("ReadCameraPoseFromYaml", &dairlib::camera::ReadCameraPoseFromYaml, py::arg("fname"));
}
}