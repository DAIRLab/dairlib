#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/swing_foot_traj_solver.h"
#include "systems/controllers/footstep_planning/alip_s2s_mpfc.h"
#include "multibody/multibody_utils.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using systems::controllers::alip_utils::Stance;
using systems::controllers::alip_utils::AlipGaitParams;
using systems::controllers::alip_utils::ResetDiscretization;
using systems::controllers::alip_utils::CalcA;
using systems::controllers::alip_utils::CalcAd;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::alip_utils::CalcAlipState;
using systems::controllers::alip_utils::CalcMassNormalizedA;
using systems::controllers::alip_utils::CalcMassNormalizedAd;
using systems::controllers::alip_utils::MakePeriodicAlipGait;
using systems::controllers::alip_utils::AlipStepToStepDynamics;
using systems::controllers::alip_utils::MassNormalizedAlipStepToStepDynamics;
using multibody::ReExpressWorldVector3InBodyYawFrame;


PYBIND11_MODULE(footstep_planning, m) {
  m.doc() = "Binding footstep planners";

  using py_rvp = py::return_value_policy;

  py::enum_<Stance>(m, "Stance")
      .value("kLeft", Stance::kLeft)
      .value("kRight", Stance::kRight);
  py::enum_<ResetDiscretization>(m, "ResetDiscretization")
      .value("kZOH", ResetDiscretization::kZOH)
      .value("kFOH", ResetDiscretization::kFOH)
      .value("kSPLIT", ResetDiscretization::kSPLIT);

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
    .def("CalcMassNormalizedA", &CalcMassNormalizedA, py::arg("com_z"))
    .def("CalcAlipStateInBodyYawFrame",
         [](const drake::multibody::MultibodyPlant<double>& plant,
            drake::systems::Context<double>* plant_context,
            const Eigen::VectorXd& state,
            std::string body_name,
            PointOnFramed stance_foot) {
        Eigen::Vector3d CoM;
        Eigen::Vector3d L;
        Eigen::Vector3d stance;

        CalcAlipState(plant, plant_context, state, {stance_foot},
          {1.0}, &CoM, &L, &stance);

        CoM = CoM - stance;

        CoM = ReExpressWorldVector3InBodyYawFrame<double>(
            plant, *plant_context, body_name, CoM);
        L = ReExpressWorldVector3InBodyYawFrame<double>(
            plant, *plant_context, body_name, L);

        Eigen::Vector4d ret;
        ret.head<2>() = CoM.head<2>();
        ret.tail<2>() = L.head<2>();
        return ret;
    }, py::arg("plant"), py::arg("plant_context"), py::arg("state"),
       py::arg("body_name"), py::arg("stance_foot"));
}


}
}