#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "cassie_utils.h"

#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"

using drake::pydrake::GetPyParam;
using drake::pydrake::pysystems::CommonScalarPack;

namespace dairlib {

PYBIND11_MODULE(cassie_utils, m){
	m.doc() = "Binding functions in the cassie_utils file";

	pybind11::module::import("pydrake.multibody.rigid_body_tree");

	m.def("makeFixedBaseCassieTreePointer", &makeFixedBaseCassieTreePointer);
}

} // namespace dairlib