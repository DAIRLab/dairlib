#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/alip_miqp.h"
#include "systems/controllers/footstep_planning/alip_multiqp.h"
#include "systems/controllers/footstep_planning/alip_utils.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using systems::controllers::AlipMultiQP;
using systems::controllers::AlipMIQP;
using systems::controllers::alip_utils::Stance;
using systems::controllers::alip_utils::AlipGaitParams;
using systems::controllers::alip_utils::ResetDiscretization;
using systems::controllers::alip_utils::CalcA;
using systems::controllers::alip_utils::CalcAd;
using systems::controllers::alip_utils::CalcMassNormalizedA;
using systems::controllers::alip_utils::CalcMassNormalizedAd;
using systems::controllers::alip_utils::MakePeriodicAlipGait;
using systems::controllers::alip_utils::AlipStepToStepDynamics;
using systems::controllers::alip_utils::MassNormalizedAlipStepToStepDynamics;


PYBIND11_MODULE(footstep_planning, m) {
  m.doc() = "Binding footstep planners";

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

  m.def("AlipStepToStepDynamics", &AlipStepToStepDynamics, py::arg("com_z"),
        py::arg("m"), py::arg("Tss"), py::arg("Tds"),
        py::arg("discretization"))
    .def("CalcAd", &CalcAd, py::arg("com_z"), py::arg("m"), py::arg("t"))
    .def("CalcA", &CalcA, py::arg("com_z"), py::arg("m"))
    .def("MassNormalizedAlipStepToStepDynamics",
        &MassNormalizedAlipStepToStepDynamics,
        py::arg("com_z"), py::arg("Tss"), py::arg("Tds"),
        py::arg("discretization"))
    .def("CalcMassNormalizedAd", &CalcMassNormalizedAd,
         py::arg("com_z"), py::arg("t"))
    .def("CalcMassNormalizedA", &CalcMassNormalizedA, py::arg("com_z"));
}


}
}