#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"

#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

using drake::pydrake::GetPyParam;
using drake::pydrake::pysystems::CommonScalarPack;

PYBIND11_MODULE(cassie_utils, m){

	m.doc() = "Binding function in the cassie_utils file";

        pybind11::module::import("pydrake.multibody.rigid_body_tree");

        m.def("makeCassieTreePointer", &makeCassieTreePointer,
                 pybind11::arg("filename") = "examples/Cassie/urdf/cassie_v2.urdf",
                 pybind11::arg("base_type") = drake::multibody::joints::kFixed);

}
}  // namespace dairlib
