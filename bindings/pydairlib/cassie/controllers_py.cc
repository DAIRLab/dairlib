#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/alip_walking_controller_diagram.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::controllers::AlipWalkingControllerDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<AlipWalkingControllerDiagram, drake::systems::Diagram<double>>(
      m, "AlipWalkingControllerFactory")
      .def(py::init<drake::multibody::MultibodyPlant<double>&, bool, bool,
                    const std::string&, const std::string&, double>(),
           py::arg("plant"), py::arg("has_double_stance"), py::arg("swing_foot_params"),
           py::arg("osc_gains_filename"), py::arg("osqp_settings_filename"), py::arg("n_knot")=0)
      .def("get_plant", &AlipWalkingControllerDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_state_input_port",
           &AlipWalkingControllerDiagram::get_state_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port",
           &AlipWalkingControllerDiagram::get_radio_input_port,
           py_rvp::reference_internal)
      .def("get_swing_foot_params_input_port",
           &AlipWalkingControllerDiagram::get_swing_foot_params_input_port,
           py_rvp::reference_internal)
      .def("get_control_output_port",
           &AlipWalkingControllerDiagram::get_control_output_port,
           py_rvp::reference_internal)
      .def("get_torque_output_port",
           &AlipWalkingControllerDiagram::get_torque_output_port,
           py_rvp::reference_internal)
      .def("get_controller_failure_output_port",
           &AlipWalkingControllerDiagram::get_controller_failure_output_port,
           py_rvp::reference_internal)
      .def("get_fsm_output_port",
           &AlipWalkingControllerDiagram::get_fsm_output_port,
           py_rvp::reference_internal)
      .def("get_swing_error_output_port",
           &AlipWalkingControllerDiagram::get_swing_error_output_port,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib