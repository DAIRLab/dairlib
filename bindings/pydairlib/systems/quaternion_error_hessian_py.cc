#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/quaternion_error_hessian.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {
PYBIND11_MODULE(quaternion_error_hessian, m) {
  m.def("hessian_of_squared_quaternion_angle_difference",
        &dairlib::systems::hessian_of_squared_quaternion_angle_difference, py::arg("quat"),
        py::arg("quat_des"));
}
}  // namespace pydairlib
}  // namespace dairlib
