#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "examples/Cassie/cassie_sysid_utils.h"

#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"

using drake::pydrake::GetPyParam;
using drake::pydrake::pysystems::CommonScalarPack;

namespace dairlib {

PYBIND11_MODULE(cassie_sysid_utils, m){
	m.doc() = "Custom simulation in drake";

	pybind11::module::import("pydrake.trajectories");
	pybind11::module::import("pydrake.multibody.rigid_body_tree");
	pybind11::module::import("pydrake.multibody.rigid_body_plant");


	m.def("getDerivativePredictionAtTime", &getDerivativePredictionAtTime)
	 .def("getSimulationTrajectoryOverTime", &getSimulationTrajectoryOverTime)
	 .def("lcmLogToTrajectory", &lcmLogToTrajectory);

}

} // namespace dairlib