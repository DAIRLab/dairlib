// python bindings

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"

#include "attic/multibody/contact_toolkit.h"

#include "attic/multibody/rigidbody_utils.h"
#include "drake/common/default_scalars.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace multibody {

namespace py = drake::pydrake::py;
using drake::pydrake::ToEigenRef;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Map;
using Eigen::Matrix3Xd;

PYBIND11_MODULE(contact_toolkit, m) {
    py::class_<ContactInfo>(m, "ContactInfo")
        .def(py::init<>())
        .def(py::init<Eigen::Matrix3Xd, std::vector<int>>());

    using namespace dairlib::multibody;

    using T = double;

    {
        using Class = ContactToolkit<T>;
        py::class_<Class>(m, "ContactToolkit", "placeholder docstring")

            .def(py::init<const RigidBodyTree<double>&, ContactInfo>(),
                py::arg("tree"), py::arg("contact_info"), "Constructor")

            .def("CalcContactJacobian", &ContactToolkit<T>::CalcContactJacobian,
                py::arg("x"), py::arg("in_terms_of_qdot"), "Contact J function");

    }
}

}  // namespace multibody
}  // namespace dairlib
