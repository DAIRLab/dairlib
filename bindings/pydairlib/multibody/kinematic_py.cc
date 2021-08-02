#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

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

PYBIND11_MODULE(kinematic, m) {
  m.doc() = "Binding for KinematicEvaluator and related classes";

  using drake::multibody::Frame;
  using drake::multibody::MultibodyPlant;
  using drake::systems::Context;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;
  using Eigen::VectorXd;

  py::class_<dairlib::multibody::KinematicEvaluator<double>>(
      m, "KinematicEvaluator")
      .def("num_full",
           &dairlib::multibody::KinematicEvaluator<double>::num_full);

  py::class_<dairlib::multibody::DistanceEvaluator<double>,
             dairlib::multibody::KinematicEvaluator<double>>(
      m, "DistanceEvaluator")
      .def(py::init<const MultibodyPlant<double>&, const Vector3d,
                    const Frame<double>&, const Vector3d, const Frame<double>&,
                    double>(),
           py::arg("plant"), py::arg("pt_A"), py::arg("frame_A"),
           py::arg("pt_B"), py::arg("frame_B"), py::arg("distance"))
      .def("EvalFull",
           overload_cast_explicit<VectorXd, const Context<double>&>(
               &dairlib::multibody::DistanceEvaluator<double>::EvalFull),
           py::arg("context"))
      .def("EvalFullTimeDerivative",
           overload_cast_explicit<VectorXd, const Context<double>&>(
               &dairlib::multibody::DistanceEvaluator<
                   double>::EvalFullTimeDerivative),
           py::arg("context"))
      .def(
          "EvalFullJacobian",
          overload_cast_explicit<MatrixXd, const Context<double>&>(
              &dairlib::multibody::DistanceEvaluator<double>::EvalFullJacobian),
          py::arg("context"))
      .def("EvalFullJacobianDotTimesV",
           overload_cast_explicit<VectorXd, const Context<double>&>(
               &dairlib::multibody::DistanceEvaluator<
                   double>::EvalFullJacobianDotTimesV),
           py::arg("context"));
}

}  // namespace pydairlib
}  // namespace dairlib
