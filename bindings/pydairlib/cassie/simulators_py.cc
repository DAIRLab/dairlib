#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


#include "drake/bindings/pydrake/pydrake_pybind.h"

#include "examples/Cassie/diagrams/cassie_sim_diagram.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using drake::pydrake::make_unowned_shared_ptr_from_raw;
using examples::CassieSimDiagram;

PYBIND11_MODULE(simulators, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<CassieSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieSimDiagram")
      .def(py::init(
               [](drake::multibody::MultibodyPlant<double>& plant,
                      const std::string& urdf, bool visualize, double mu,
                      double stiffness, double dissipation_rate) {
                return std::make_unique<CassieSimDiagram>(
                   make_unowned_shared_ptr_from_raw(&plant), urdf, visualize,
                      mu);
            }),
           py::arg("plant"), py::arg("urdf"), py::arg("visualize"), py::arg("mu"), py::arg("stiffness"),
           py::arg("dissipation_rate"))
      .def("get_plant", &CassieSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_input_port_actuation",
           &CassieSimDiagram::get_input_port_actuation,
           py_rvp::reference_internal)
      .def("get_input_port_radio", &CassieSimDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_state", &CassieSimDiagram::get_output_port_state,
           py_rvp::reference_internal)
      .def("get_output_port_cassie_out",
           &CassieSimDiagram::get_output_port_cassie_out,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib