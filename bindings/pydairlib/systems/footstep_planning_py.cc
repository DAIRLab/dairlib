#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "systems/controllers/footstep_planning/swing_foot_traj_solver.h"
#include "systems/controllers/footstep_planning/alip_s2s_mpfc.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

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
using systems::controllers::SwingFootTrajSolver;
using systems::controllers::alip_s2s_mpfc_solution;
using systems::controllers::alip_s2s_mpfc_params;
using systems::controllers::alip_s2s_mpfc_input;
using systems::controllers::AlipS2SMPFC;


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

  py::class_<SwingFootTrajSolver>(m, "SwingFootTrajSolver")
      .def(py::init<>())
      .def("AdaptSwingFootTraj", &SwingFootTrajSolver::AdaptSwingFootTraj,
           py::arg("prev_traj"), py::arg("prev_time"), py::arg("t_start"),
           py::arg("t_end"), py::arg("swing_foot_clearance"),
           py::arg("z_vel_final"), py::arg("z_pos_final_offset"),
           py::arg("initial_pos"), py::arg("footstep_target"));

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

  py::class_<alip_s2s_mpfc_solution>(m, "AlipS2SMpfcSolution")
      // Constructor
      .def(py::init<>())
      .def_readwrite("pp", &alip_s2s_mpfc_solution::pp)
      .def_readwrite("xx", &alip_s2s_mpfc_solution::xx)
      .def_readwrite("ee", &alip_s2s_mpfc_solution::ee)
      .def_readwrite("mu", &alip_s2s_mpfc_solution::mu)
      .def_readwrite("t_sol", &alip_s2s_mpfc_solution::t_sol)
      .def_readwrite("u_sol", &alip_s2s_mpfc_solution::u_sol)
      .def_readwrite("success", &alip_s2s_mpfc_solution::success)
      .def_readwrite("total_cost", &alip_s2s_mpfc_solution::total_cost)
      .def_readwrite("footstep_cost", &alip_s2s_mpfc_solution::footstep_cost)
      .def_readwrite("time_reg_cost", &alip_s2s_mpfc_solution::time_reg_cost)
      .def_readwrite("input_reg_cost", &alip_s2s_mpfc_solution::input_reg_cost)
      .def_readwrite("final_cost", &alip_s2s_mpfc_solution::final_cost)
      .def_readwrite("state_cost", &alip_s2s_mpfc_solution::state_cost)
      .def_readwrite("soft_constraint_cost", &alip_s2s_mpfc_solution::soft_constraint_cost)
      .def_readwrite("t_nom", &alip_s2s_mpfc_solution::t_nom)
      .def_readwrite("total_time", &alip_s2s_mpfc_solution::total_time)
      .def_readwrite("optimizer_time", &alip_s2s_mpfc_solution::optimizer_time)
      .def_readwrite("desired_velocity", &alip_s2s_mpfc_solution::desired_velocity)
      .def_readwrite("solution_result", &alip_s2s_mpfc_solution::solution_result)
      .def_readwrite("input_footholds", &alip_s2s_mpfc_solution::input_footholds);

  py::class_<dairlib::systems::controllers::alip_s2s_mpfc_params>(m, "AlipS2SMpfcParams")
      .def(py::init<>())
      .def_readwrite("gait_params", &alip_s2s_mpfc_params::gait_params)
      .def_readwrite("nmodes", &alip_s2s_mpfc_params::nmodes)
      .def_readwrite("tmin", &alip_s2s_mpfc_params::tmin)
      .def_readwrite("tmax", &alip_s2s_mpfc_params::tmax)
      .def_readwrite("soft_constraint_cost", &alip_s2s_mpfc_params::soft_constraint_cost)
      .def_readwrite("time_regularization", &alip_s2s_mpfc_params::time_regularization)
      .def_readwrite("com_pos_bound", &alip_s2s_mpfc_params::com_pos_bound)
      .def_readwrite("com_vel_bound", &alip_s2s_mpfc_params::com_vel_bound)
      .def_readwrite("Q", &alip_s2s_mpfc_params::Q)
      .def_readwrite("R", &alip_s2s_mpfc_params::R)
      .def_readwrite("Qf", &alip_s2s_mpfc_params::Qf)
      .def_readwrite("solver_options", &alip_s2s_mpfc_params::solver_options)
      .def_readwrite("ncqp_solver_options_path", &alip_s2s_mpfc_params::ncqp_solver_options_path)
      .def_readwrite("umax", &dairlib::systems::controllers::alip_s2s_mpfc_params::umax)
      .def_readwrite("ankle_torque_regularization", &alip_s2s_mpfc_params::ankle_torque_regularization)
      .def_readwrite("tracking_cost_type", &alip_s2s_mpfc_params::tracking_cost_type)
      .def_readwrite("miqp", &alip_s2s_mpfc_params::miqp);

  py::class_<alip_s2s_mpfc_input>(m, "AlipS2SMpfcInput")
      .def(py::init<>())
      .def_readwrite("x", &alip_s2s_mpfc_input::x)
      .def_readwrite("p", &alip_s2s_mpfc_input::p)
      .def_readwrite("t", &alip_s2s_mpfc_input::t)
      .def_readwrite("vdes", &alip_s2s_mpfc_input::vdes)
      .def_readwrite("tmin", &alip_s2s_mpfc_input::tmin)
      .def_readwrite("tmax", &alip_s2s_mpfc_input::tmax)
      .def_readwrite("stance", &alip_s2s_mpfc_input::stance)
      .def_readwrite("footholds", &alip_s2s_mpfc_input::footholds)
      .def_readwrite("p_prev_stance", &alip_s2s_mpfc_input::p_prev_stance);

  m.def("make_alip_s2s_mpfc_params_from_yaml",
        &dairlib::systems::controllers::MakeAlipS2SMPFCParamsFromYaml,
        py::arg("gains_yaml_path"),
        py::arg("solver_options_yaml_path"),
        py::arg("plant"),
        py::arg("context"),
        "Create ALIP S2S MPFC parameters from YAML configuration files");

  py::class_<AlipS2SMPFC>(m, "AlipS2SMPFC")
      .def(py::init<alip_s2s_mpfc_params>(), py::arg("params"))
      .def("Solve", &AlipS2SMPFC::SolveFromInput, py::arg("input"));
}


}
}