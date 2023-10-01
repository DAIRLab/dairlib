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

  py::enum_<Stance>(m, "Stance")
      .value("kLeft", Stance::kLeft)
      .value("kRight", Stance::kRight);
  py::enum_<ResetDiscretization>(m, "ResetDiscretization")
      .value("kZOH", ResetDiscretization::kZOH)
      .value("kFOH", ResetDiscretization::kFOH);

  py::class_<AlipGaitParams>(
      m, "AlipGaitParams")
      .def(py::init<double, double, double, double, double, Eigen::Vector2d,
           Stance, ResetDiscretization>(),
           py::arg("height"),
           py::arg("mass"),
           py::arg("single_stance_duration"),
           py::arg("double_stance_duration"),
           py::arg("stance_width"),
           py::arg("desired_velocity"),
           py::arg("initial_stance_foot"),
           py::arg("reset_discretization_method"))
      .def_readwrite("height",
                      &AlipGaitParams::height)
      .def_readwrite("mass",
                      &AlipGaitParams::mass)
      .def_readwrite("single_stance_duration",
                      &AlipGaitParams::single_stance_duration)
      .def_readwrite("double_stance_duration",
                      &AlipGaitParams::double_stance_duration)
      .def_readwrite("stance_width",
                      &AlipGaitParams::stance_width)
      .def_readwrite("desired_velocity",
                      &AlipGaitParams::desired_velocity)
      .def_readwrite("initial_stance_foot",
                      &AlipGaitParams::initial_stance_foot)
      .def_readwrite("reset_discretization_method",
                      &AlipGaitParams::reset_discretization_method);

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

  py::class_<AlipMIQP>(
      m, "AlipMIQP")
      .def(
          py::init<double, double, int, ResetDiscretization, int>(),
          py::arg("m"), py::arg("H"), py::arg("nk"),
          py::arg("reset_discretization_method"), py::arg("nmodes"))
      .def("AddFootholds", &AlipMIQP::AddFootholds)
      .def("AddMode", &AlipMIQP::AddMode)
      .def("AddInputCost", &AlipMIQP::AddInputCost)
      .def("ActivateInitialTimeEqualityConstraint", &AlipMIQP::ActivateInitialTimeEqualityConstraint)
      .def("UpdateMaximumCurrentStanceTime", &AlipMIQP::UpdateMaximumCurrentStanceTime)
      .def("UpdateTrackingCost", &AlipMIQP::UpdateTrackingCost)
      .def("AddTrackingCost", &AlipMIQP::AddTrackingCost)
      .def("Build", py::overload_cast<>(&AlipMIQP::Build))
      .def("UpdateFootholds", &AlipMIQP::UpdateFootholds)
      .def("MakeXdesTrajForVdes", &AlipMIQP::MakeXdesTrajForVdes)
      .def("UpdateNominalStanceTime", &AlipMIQP::UpdateNominalStanceTime)
      .def("SetInputLimit", &AlipMIQP::SetInputLimit)
      .def("SetMinimumStanceTime", &AlipMIQP::SetMinimumStanceTime)
      .def("SetMaximumStanceTime", &AlipMIQP::SetMaximumStanceTime)
      .def("SetDoubleSupportTime", &AlipMIQP::SetDoubleSupportTime)
      .def("CalcOptimalFootstepPlan", &AlipMIQP::CalcOptimalFootstepPlan)
      .def("GetFootstepSolution", &AlipMIQP::GetFootstepSolution)
      .def("GetStateSolution", &AlipMIQP::GetStateSolution)
      .def("GetInputSolution", &AlipMIQP::GetInputSolution)
      .def("GetTimingSolution", &AlipMIQP::GetTimingSolution)
      .def("GetFootstepGuess", &AlipMIQP::GetFootstepGuess)
      .def("GetStateGuess", &AlipMIQP::GetStateGuess)
      .def("GetInputGuess", &AlipMIQP::GetInputGuess)
      .def("GetTimingSGuess", &AlipMIQP::GetTimingGuess)
      .def("GetStateDesired", &AlipMIQP::GetTimingDesired)
      .def("GetDesiredInput", &AlipMIQP::GetTimingDesired)
      .def("GetTimingDesired", &AlipMIQP::GetTimingDesired)
      .def("GetDesiredFootstep", &AlipMIQP::GetTimingDesired)
      .def("get_prog", &AlipMIQP::get_prog, py_rvp::reference_internal)
      .def("get_solution", &AlipMIQP::get_solution, py_rvp::reference_internal)
      .def("nmodes", &AlipMIQP::nmodes)
      .def("nknots", &AlipMIQP::nknots);

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
      .def("get_plant", &MpfcOscDiagram::get_plant, py_rvp::reference_internal);

  m.def("AlipStepToStepDynamics", &AlipStepToStepDynamics, py::arg("com_z"),
        py::arg("m"), py::arg("Tss"), py::arg("Tds"),
        py::arg("discretization"))
    .def("CalcAd", &CalcAd, py::arg("com_z"), py::arg("m"), py::arg(""));
}


}
}