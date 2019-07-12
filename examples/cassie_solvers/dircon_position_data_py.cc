// python bindings

#include <limits>
#include <vector>
#include <memory>

#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "attic/systems/trajectory_optimization/dircon_position_data.h"

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace dairlib {

namespace py = drake::pydrake::py;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Map;
using Eigen::Matrix3Xd;
using Eigen::Vector2d;
using Eigen::Vector3d;

PYBIND11_MODULE(dircon_position_data, m) {

    using namespace dairlib;

    using T = double;

    {
        using Class = DirconPositionData<T>;
        py::class_<Class>(m, "DirconPositionData", "placeholder docstring")

            .def(py::init<const RigidBodyTree<double>&, int, Vector3d, bool>(),
                py::arg("tree"), py::arg("bodyIdx"), py::arg("pt"), py::arg("isXZ"), "Constructor")

            .def("Jdotv", &DirconPositionData<T>::Jdotv,
                py::arg("cache"), "Contact J function");

    }
}

}  // namespace dairlib
