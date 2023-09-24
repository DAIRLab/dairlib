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
using systems::controllers::alip_utils::Stance;
using systems::controllers::alip_utils::ResetDiscretization;
using perceptive_locomotion::MpfcOscDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding generic controllers";

  using py_rvp = py::return_value_policy;

  py::enum_<Stance>(m, "Stance")
      .value("kLeft", Stance::kLeft)
      .value("kRight", Stance::kRight);
  py::enum_<ResetDiscretization>(m, "ResetDiscretization")
      .value("kZOH", ResetDiscretization::kZOH)
      .value("kFOH", ResetDiscretization::kFOH);

  py::class_<AlipMultiQP>(
      m, "AlipMultiQP")
      .def(
          py::init<double, double, int, ResetDiscretization, int>(),
          py::arg("m"), py::arg("H"), py::arg("nk"),
          py::arg("reset_discretization_method"), py::arg("nmodes"))
      .def("AddFootholds", &AlipMultiQP::AddFootholds)
      .def("AddMode", &AlipMultiQP::AddMode)
      .def("AddInputCost", &AlipMultiQP::AddInputCost)
      .def("ActivateInitialTimeEqualityConstraint", &AlipMultiQP::ActivateInitialTimeEqualityConstraint)
      .def("UpdateMaximumCurrentStanceTime", &AlipMultiQP::UpdateMaximumCurrentStanceTime)
      .def("UpdateTrackingCost", &AlipMultiQP::UpdateTrackingCost)
      .def("AddTrackingCost", &AlipMultiQP::AddTrackingCost)
      .def("Build", py::overload_cast<>(&AlipMultiQP::Build))
      .def("UpdateFootholds", &AlipMultiQP::UpdateFootholds)
      .def("MakeXdesTrajForVdes", &AlipMultiQP::MakeXdesTrajForVdes)
      .def("UpdateNominalStanceTime", &AlipMultiQP::UpdateNominalStanceTime)
      .def("SetInputLimit", &AlipMultiQP::SetInputLimit)
      .def("SetMinimumStanceTime", &AlipMultiQP::SetMinimumStanceTime)
      .def("SetMaximumStanceTime", &AlipMultiQP::SetMaximumStanceTime)
      .def("SetDoubleSupportTime", &AlipMultiQP::SetDoubleSupportTime)
      .def("CalcOptimalFootstepPlan", &AlipMultiQP::CalcOptimalFootstepPlan)
      .def("GetFootstepSolution", &AlipMultiQP::GetFootstepSolution)
      .def("GetStateSolution", &AlipMultiQP::GetStateSolution)
      .def("GetInputSolution", &AlipMultiQP::GetInputSolution)
      .def("GetTimingSolution", &AlipMultiQP::GetTimingSolution)
      .def("GetFootstepGuess", &AlipMultiQP::GetFootstepGuess)
      .def("GetStateGuess", &AlipMultiQP::GetStateGuess)
      .def("GetInputGuess", &AlipMultiQP::GetInputGuess)
      .def("GetTimingSGuess", &AlipMultiQP::GetTimingGuess)
      .def("GetStateDesired", &AlipMultiQP::GetTimingDesired)
      .def("GetDesiredInput", &AlipMultiQP::GetTimingDesired)
      .def("GetTimingDesired", &AlipMultiQP::GetTimingDesired)
      .def("GetDesiredFootstep", &AlipMultiQP::GetTimingDesired)
      .def("get_prog", &AlipMultiQP::get_prog, py_rvp::reference_internal)
      .def("get_solution", &AlipMultiQP::get_solution, py_rvp::reference_internal)
      .def("nmodes", &AlipMultiQP::nmodes)
      .def("nknots", &AlipMultiQP::nknots);

  py::class_<MpfcOscDiagram>(
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
      .def("get_output_port_u_cmd",
           &MpfcOscDiagram::get_output_port_u_cmd,
           py_rvp::reference_internal)
      .def("get_output_port_fsm",
           &MpfcOscDiagram::get_output_port_fsm,
           py_rvp::reference_internal)
      .def("get_plant", &MpfcOscDiagram::get_plant, py_rvp::reference_internal);

  m.def("MakeMinSnapTrajFromWaypoints",
        &minsnap::MakeMinSnapTrajFromWaypoints, py::arg("waypoints"),
        py::arg("breaks"), py::arg("initial_velocity"), py::arg("final_velocity"));
}


}
}