#include <cstring>

#include "bindings/pydairlib/lcm/lcm_py_bind_cpp_serializers.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace dairlib {
namespace pydairlib {

using dairlib::pydairlib::BindCppSerializers;

// pybind11 trampoline class to permit overriding virtual functions in
// Python.
PYBIND11_MODULE(lcm_py, m) {
//  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  // Bind C++ serializers.
  BindCppSerializers();
}

}  // namespace pydrake
}  // namespace drake