// python bindings for lcm_log_utils. Single function "parseLCMOutputLog" outputs a tuple containing t, x and u

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"

#include "attic/multibody/lcm_log_utils.h"

namespace dairlib {
namespace multibody {

namespace py = drake::pydrake::py;
using drake::pydrake::ToEigenRef;
using Eigen::VectorXd;
using Eigen::MatrixXd;

PYBIND11_MODULE(lcm_log_utils, m){
	m.doc() = "Binding function in the lcm_log_utils file";

	pybind11::module::import("pydrake.attic.multibody.rigid_body_tree");

	m.def("parseLcmOutputLog", 
            [](const RigidBodyTree<double>& tree, std::string file,
               std::string channel, double duration) {
              VectorXd t;
              MatrixXd x, u;
              parseLcmOutputLog(tree, file, channel, &t, &x, &u, duration);
              return py::make_tuple(t, x, u);
            });
}
}  // namespace multibody
}  // namespace dairlib
