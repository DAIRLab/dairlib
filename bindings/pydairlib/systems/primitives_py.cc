#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/primitives/subvector_pass_through.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

// Borrowed from Drake bindings, turns out writing bindings for templated class
// and functions is a huge pain.
//
// Implementation for `overload_cast_explicit`. We
// must use this structure so that we can constrain what is inferred. Otherwise,
// the ambiguity confuses the compiler.
template <typename Return, typename... Args>
struct overload_cast_impl {
  auto operator()(Return (*func)(Args...)) const { return func; }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...)) const {
    return method;
  }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...) const) const {
    return method;
  }
};

/// Provides option to provide explicit signature when
/// `py::overload_cast<Args...>` fails to infer the Return argument.
template <typename Return, typename... Args>
constexpr auto overload_cast_explicit = overload_cast_impl<Return, Args...>{};

PYBIND11_MODULE(primitives, m) {
  m.doc() = "Binding robot lcm systems";

  using drake::multibody::MultibodyPlant;

  py::class_<systems::SubvectorPassThrough<double>, drake::systems::LeafSystem<double>>(m, "SubvectorPassThrough")
      .def(py::init<int, int, int>());
}

}  // namespace pydairlib
}  // namespace dairlib
