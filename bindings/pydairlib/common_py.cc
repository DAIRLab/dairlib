#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "common/find_resource.h"

namespace py = pybind11;

namespace dairlib  {
namespace pydairlib {

PYBIND11_MODULE(common, m) {
  m.doc() = "Bindings for //common:common";

  m.def("FindResourceOrThrow", &FindResourceOrThrow,
      "Attempts to locate a Dairlib (not Drake) resource named by the given path string. "
      "The path refers to the relative path within the repository, "
      "e.g., examples/Cassie/urdf/cassie.urdf. Raises an exception "
      "if the resource was not found.",
      py::arg("resource_path"));
}

}  // namespace pydairlib
}  // namespace dairlib
