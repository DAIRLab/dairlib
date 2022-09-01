#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/alip_minlp.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using systems::controllers::AlipMINLP;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding generic controllers";

  using py_rvp = py::return_value_policy;

  py::class_<AlipMINLP>(
      m, "AlipMINLP")
      .def(py::init<double, double>(), py::arg("m"), py::arg("H"))
      .def("AddFootholds", &AlipMINLP::AddFootholds)
      .def("AddMode", &AlipMINLP::AddMode)
      .def("AddInputCost", &AlipMINLP::AddInputCost)
      .def("ActivateInitialTimeConstraint", &AlipMINLP::ActivateInitialTimeConstraint)
      .def("DeactivateInitialTimeConstraint", &AlipMINLP::DeactivateInitialTimeConstraint)
      .def("UpdateTrackingCost", &AlipMINLP::UpdateTrackingCost)
      .def("AddTrackingCost", &AlipMINLP::AddTrackingCost)
      .def("Build", &AlipMINLP::Build)
      .def("ChangeFootholds", &AlipMINLP::ChangeFootholds)
      .def("MakeXdesTrajForVdes", &AlipMINLP::MakeXdesTrajForVdes)
      .def("MakeXdesTrajForCurrentStep", &AlipMINLP::MakeXdesTrajForCurrentStep)
      .def("SetNominalStanceTime", &AlipMINLP::SetNominalStanceTime)
      .def("CalcOptimalFootstepPlan", &AlipMINLP::CalcOptimalFootstepPlan)
      .def("GetFootstepSolution", &AlipMINLP::GetFootstepSolution)
      .def("GetStateSolution", &AlipMINLP::GetStateSolution)
      .def("GetInputSolution", &AlipMINLP::GetInputSolution)
      .def("GetTimingSolution", &AlipMINLP::GetTimingSolution)
      .def("nmodes", &AlipMINLP::nmodes)
      .def("nknots", &AlipMINLP::nknots);
}


}
}