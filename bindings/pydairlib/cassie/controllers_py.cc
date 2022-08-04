#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/alip_walking_controller_diagram.h"
#include "examples/Cassie/diagrams/footstep_target_controller_diagram.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::controllers::AlipWalkingControllerDiagram;
using examples::controllers::FootstepTargetControllerDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<AlipWalkingControllerDiagram, drake::systems::Diagram<double>>(
      m, "AlipWalkingControllerFactory")
      .def(py::init<drake::multibody::MultibodyPlant<double>&, bool,
                    const std::string&, const std::string&>(),
           py::arg("plant"), py::arg("has_double_stance"),
           py::arg("osc_gains_filename"), py::arg("osqp_settings_filename"))
      .def("get_plant", &AlipWalkingControllerDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_state_input_port",
           &AlipWalkingControllerDiagram::get_state_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port",
           &AlipWalkingControllerDiagram::get_radio_input_port,
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
          py_rvp::reference_internal);

  py::class_<FootstepTargetControllerDiagram, drake::systems::Diagram<double>>(
      m, "FootstepTargetWalkingControllerFactory")
      .def(py::init<drake::multibody::MultibodyPlant<double>&, bool,
              const std::string&, const std::string&, double>(),
      py::arg("plant"), py::arg("has_double_stance"),
      py::arg("osc_gains_filename"),
      py::arg("osqp_settings_filename"),
      py::arg("single_stance_time_override"))
      .def("get_plant", &FootstepTargetControllerDiagram::get_plant,
          py_rvp::reference_internal)
      .def("get_state_input_port",
          &FootstepTargetControllerDiagram::get_state_input_port,
          py_rvp::reference_internal)
      .def("get_radio_input_port",
          &FootstepTargetControllerDiagram::get_radio_input_port,
          py_rvp::reference_internal)
      .def("get_footstep_target_input_port",
          &FootstepTargetControllerDiagram::get_footstep_target_input_port,
          py_rvp::reference_internal)
      .def("get_control_output_port",
          &FootstepTargetControllerDiagram::get_control_output_port,
          py_rvp::reference_internal)
      .def("get_torque_output_port",
          &FootstepTargetControllerDiagram::get_torque_output_port,
          py_rvp::reference_internal)
      .def("get_fsm_output_port",
          &FootstepTargetControllerDiagram::get_fsm_output_port,
          py_rvp::reference_internal)
      .def("get_alip_target_footstep_port",
           &FootstepTargetControllerDiagram::get_alip_target_footstep_port,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib