// python bindings

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"

#include "attic/multibody/multibody_solvers.h"

#include "attic/multibody/rigidbody_utils.h"
#include "drake/multibody/rigid_body_tree.h"

#include "attic/multibody/contact_toolkit.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace multibody {

namespace py = drake::pydrake::py;
using drake::pydrake::ToEigenRef;
using Eigen::VectorXd;
using Eigen::MatrixXd;

PYBIND11_MODULE(multibody_solvers, m) {

    using namespace dairlib::multibody;

    {
        using Class = FixedPointConstraint;
        py::class_<Class>(m, "FixedPointConstraint", "placeholder docstring")
            .def(py::init<const RigidBodyTree<double>&, ContactInfo>(),
                py::arg("tree"), py::arg("contact_info"), "Constructor");

            m.def("DoEval", 
                [](const Eigen::Ref<const Eigen::VectorXd>& q_u_l, Eigen::VectorXd* y) {
                   FixedPointConstraint::DoEval(q_u_l, y);
                   return py::make_tuple(q_u_l, y);
                });
    }
}
}  // namespace multibody
}  // namespace dairlib

