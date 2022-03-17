#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "examples/Cassie/osc_run/osc_running_gains.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(dairlib_yaml, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<OSCGains>(m, "OSCGains")
      .def(py::init<>());
  py::class_<OSCRunningGains>(m, "OSCRunningGains")
      .def(py::init<>());
}
}  // namespace pydairlib
}  // namespace dairlib