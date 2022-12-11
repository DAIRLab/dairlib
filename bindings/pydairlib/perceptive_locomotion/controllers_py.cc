#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/alip_minlp.h"
#include "systems/controllers/footstep_planning/alip_utils.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using systems::controllers::AlipMINLP;
using systems::controllers::alip_utils::Stance;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding generic controllers";

  using py_rvp = py::return_value_policy;

  py::enum_<Stance>(m, "Stance")
      .value("kLeft", Stance::kLeft)
      .value("kRight", Stance::kRight);

  py::class_<AlipMINLP>(
      m, "AlipMINLP")
      .def(py::init<double, double>(), py::arg("m"), py::arg("H"))
      .def("AddFootholds", &AlipMINLP::AddFootholds)
      .def("AddMode", &AlipMINLP::AddMode)
      .def("AddInputCost", &AlipMINLP::AddInputCost)
      .def("ActivateInitialTimeEqualityConstraint", &AlipMINLP::ActivateInitialTimeEqualityConstraint)
      .def("UpdateMaximumCurrentStanceTime", &AlipMINLP::UpdateMaximumCurrentStanceTime)
      .def("UpdateTrackingCost", &AlipMINLP::UpdateTrackingCost)
      .def("AddTrackingCost", &AlipMINLP::AddTrackingCost)
      .def("Build", py::overload_cast<>(&AlipMINLP::Build))
      .def("UpdateFootholds", &AlipMINLP::UpdateFootholds)
      .def("MakeXdesTrajForVdes", &AlipMINLP::MakeXdesTrajForVdes)
      .def("UpdateNominalStanceTime", &AlipMINLP::UpdateNominalStanceTime)
      .def("SetInputLimit", &AlipMINLP::SetInputLimit)
      .def("SetMinimumStanceTime", &AlipMINLP::SetMinimumStanceTime)
      .def("SetMaximumStanceTime", &AlipMINLP::SetMaximumStanceTime)
      .def("SetDoubleSupportTime", &AlipMINLP::SetDoubleSupportTime)
      .def("CalcOptimalFootstepPlan", &AlipMINLP::CalcOptimalFootstepPlan)
      .def("GetFootstepSolution", &AlipMINLP::GetFootstepSolution)
      .def("GetStateSolution", &AlipMINLP::GetStateSolution)
      .def("GetInputSolution", &AlipMINLP::GetInputSolution)
      .def("GetTimingSolution", &AlipMINLP::GetTimingSolution)
      .def("GetFootstepGuess", &AlipMINLP::GetFootstepGuess)
      .def("GetStateGuess", &AlipMINLP::GetStateGuess)
      .def("GetInputGuess", &AlipMINLP::GetInputGuess)
      .def("GetTimingSGuess", &AlipMINLP::GetTimingGuess)
      .def("GetDesiredState", &AlipMINLP::GetDesiredTiming)
      .def("GetDesiredInput", &AlipMINLP::GetDesiredTiming)
      .def("GetDesiredTiming", &AlipMINLP::GetDesiredTiming)
      .def("GetDesiredFootstep", &AlipMINLP::GetDesiredTiming)
      .def("get_prog", &AlipMINLP::get_prog, py_rvp::reference_internal)
      .def("get_solution", &AlipMINLP::get_solution, py_rvp::reference_internal)
      .def("nmodes", &AlipMINLP::nmodes)
      .def("nknots", &AlipMINLP::nknots);
}


}
}