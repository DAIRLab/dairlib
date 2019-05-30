// python bindings for lcm_log_utils. Single function "parseLCMOutputLog" outputs a tuple containing t, x and u parsed from a .log file.

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"

#include "attic/multibody/lcm_log_utils.h"

namespace dairlib {
namespace multibody {

namespace py = drake::pydrake::py;
using drake::pydrake::ToEigenRef;

PYBIND11_MODULE(lcm_log_utils, m){
	m.doc() = "Binding function in the lcm_log_utils file";

	pybind11::module::import("pydrake.multibody.rigid_body_tree");

        // arguments and their types must be specified
	m.def("parseLcmOutputLog", 
            [](const RigidBodyTree<double>& tree, std::string file,
               std::string channel, Eigen::VectorXd* t, Eigen::MatrixXd* x,
               Eigen::MatrixXd* u, double duration) {
              parseLcmOutputLog(tree, file, channel, t, x, u, duration);
              return py::make_tuple(*t, *x, *u);
            });
}
}  // namespace multibody
}  // namespace dairlib
