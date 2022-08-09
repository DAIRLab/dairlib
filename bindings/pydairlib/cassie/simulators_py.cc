#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/cassie_sim_diagram.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::CassieSimDiagram;

PYBIND11_MODULE(simulators, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<dairlib::examples::CassieSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieSimDiagram")
      .def(py::init<
               std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
               const std::string&, bool, double, double, double>(),
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