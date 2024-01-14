#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/symbolic_types_pybind.h"

#include "solvers/optimization_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using drake::solvers::MathematicalProgram;

using solvers::LinearBigMConstraint;
using solvers::LinearBigMEqualityConstraint;
using py_rvp = py::return_value_policy;

PYBIND11_MODULE(optimization_utils, m) {
  py::class_<LinearBigMConstraint>(m, "LinearBigMConstraint")
      .def(py::init<MathematicalProgram&, const Eigen::MatrixXd&,
                    const Eigen::VectorXd&, double,
                    const drake::solvers::VectorXDecisionVariable&,
                    const drake::solvers::DecisionVariable&>(),
                    py::arg("prog"), py::arg("A"), py::arg("b"), py::arg("M"),
                    py::arg("x"), py::arg("z"))
      .def("UpdateCoefficients", &LinearBigMConstraint::UpdateCoefficients)
      .def("deactivate", py::overload_cast<>(&LinearBigMConstraint::deactivate))
      .def("deactivate", py::overload_cast<MathematicalProgram&>(
           &LinearBigMConstraint::deactivate))
      .def("get_constraint_binding", &LinearBigMConstraint::get_constraint_binding,
           py_rvp::reference_internal)
      .def("CheckSatisfiedIfActive", &LinearBigMConstraint::CheckSatisfiedIfActive);

  py::class_<LinearBigMEqualityConstraint>(m, "LinearBigMEqualityConstraint")
      .def(py::init<MathematicalProgram&, const Eigen::MatrixXd&,
                    const Eigen::VectorXd&, double,
                    const drake::solvers::VectorXDecisionVariable&,
                    const drake::solvers::DecisionVariable&>())
      .def("UpdateCoefficients", &LinearBigMEqualityConstraint::UpdateCoefficients)
      .def("deactivate", py::overload_cast<>(&LinearBigMEqualityConstraint::deactivate))
      .def("deactivate", py::overload_cast<MathematicalProgram&>(
          &LinearBigMEqualityConstraint::deactivate));
}

}
}