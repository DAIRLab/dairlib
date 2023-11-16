#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/alip_miqp.h"
#include "systems/controllers/footstep_planning/alip_multiqp.h"
#include "systems/controllers/footstep_planning/alip_utils.h"
#include "systems/controllers/minimum_snap_trajectory_generation.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using systems::controllers::AlipMultiQP;
using systems::controllers::AlipMIQP;
using systems::controllers::alip_utils::CalcAd;
using systems::controllers::alip_utils::Stance;
using systems::controllers::alip_utils::AlipGaitParams;
using systems::controllers::alip_utils::ResetDiscretization;
using systems::controllers::alip_utils::MakePeriodicAlipGait;
using systems::controllers::alip_utils::AlipStepToStepDynamics;
using perceptive_locomotion::MpfcOscDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding generic controllers";

  using py_rvp = py::return_value_policy;

  py::class_<MpfcOscDiagram, drake::systems::Diagram<double>>(
      m, "MpfcOscDiagram")
      .def(py::init<drake::multibody::MultibodyPlant<double>&,
           const std::string&, const std::string&, const std::string&>(),
           py::arg("plant"), py::arg("osc_gains_filename"),
           py::arg("mpc_gains_filename"), py::arg("oscp_settings_filename"))
      .def("get_input_port_state",
           &MpfcOscDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_footstep_command",
           &MpfcOscDiagram::get_input_port_footstep_command,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &MpfcOscDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_actuation",
           &MpfcOscDiagram::get_output_port_actuation,
           py_rvp::reference_internal)
      .def("get_output_port_fsm",
           &MpfcOscDiagram::get_output_port_fsm,
           py_rvp::reference_internal)
      .def("get_output_port_alip",
           &MpfcOscDiagram::get_output_port_alip,
           py_rvp::reference_internal)
      .def("get_output_port_switching_time",
           &MpfcOscDiagram::get_output_port_switching_time,
           py_rvp::reference_internal)
      .def("get_plant", &MpfcOscDiagram::get_plant, py_rvp::reference_internal)
      .def("SetSwingFootPositionAtLiftoff",
           &MpfcOscDiagram::SetSwingFootPositionAtLiftoff);
}


}
}