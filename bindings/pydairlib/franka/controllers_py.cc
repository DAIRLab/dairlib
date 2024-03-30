#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "examples/franka/diagrams/franka_c3_controller_diagram.h"
#include "examples/franka/diagrams/franka_osc_controller_diagram.h"
#include "solvers/c3_options.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::controllers::FrankaC3ControllerDiagram;
using examples::controllers::FrankaOSCControllerDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding controller factories for plate balancing demo";

  using py_rvp = py::return_value_policy;
  py::module::import("pydrake.math");

  py::class_<FrankaOSCControllerDiagram, drake::systems::Diagram<double>>(
      m, "FrankaOSCControllerDiagram")
      .def(py::init<const std::string&, const std::string&,
                    drake::lcm::DrakeLcm*>(),
           py::arg("controller_settings"), py::arg("lcm_channels"),
           py::arg("lcm"))
      .def("get_input_port_robot_state",
           &FrankaOSCControllerDiagram::get_input_port_robot_state,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_position",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_position,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_orientation",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_orientation,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_force",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_force,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &FrankaOSCControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_robot_input",
           &FrankaOSCControllerDiagram::get_output_port_robot_input,
           py_rvp::reference_internal);

  py::class_<FrankaC3ControllerDiagram, drake::systems::Diagram<double>>(
      m, "FrankaC3ControllerDiagram")
      .def(py::init<const std::string&, const C3Options, const std::string&,
                    drake::lcm::DrakeLcm*>(),
           py::arg("controller_settings"), py::arg("c3_options"),
           py::arg("lcm_channels"), py::arg("lcm"))
      .def("get_input_port_robot_state",
           &FrankaC3ControllerDiagram::get_input_port_robot_state,
           py_rvp::reference_internal)
      .def("get_input_port_object_state",
           &FrankaC3ControllerDiagram::get_input_port_object_state,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &FrankaC3ControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_mpc_plan",
           &FrankaC3ControllerDiagram::get_output_port_mpc_plan,
           py_rvp::reference_internal);

  py::class_<C3Options>(m, "C3Options")
      .def(py::init<>())
      .def_readwrite("admm_iter", &C3Options::admm_iter)
      .def_readwrite("rho", &C3Options::rho)
      .def_readwrite("rho_scale", &C3Options::rho_scale)
      .def_readwrite("num_threads", &C3Options::num_threads)
      .def_readwrite("delta_option", &C3Options::delta_option)
      .def_readwrite("projection_type", &C3Options::projection_type)
      .def_readwrite("contact_model", &C3Options::contact_model)
      .def_readwrite("warm_start", &C3Options::warm_start)
      .def_readwrite("use_predicted_x0", &C3Options::use_predicted_x0)
      .def_readwrite("solve_time_filter_alpha", &C3Options::solve_time_filter_alpha)
      .def_readwrite("publish_frequency", &C3Options::publish_frequency)
      .def_readwrite("world_x_limits", &C3Options::workspace_limits)
      .def_readwrite("u_horizontal_limits", &C3Options::u_horizontal_limits)
      .def_readwrite("u_vertical_limits", &C3Options::u_vertical_limits)
      .def_readwrite("workspace_margins", &C3Options::workspace_margins)
      .def_readwrite("N", &C3Options::N)
      .def_readwrite("gamma", &C3Options::gamma)
      .def_readwrite("q_vector", &C3Options::q_vector)
      .def_readwrite("r_vector", &C3Options::r_vector)
      .def_readwrite("g_vector", &C3Options::g_vector)
      .def_readwrite("g_x", &C3Options::g_x)
      .def_readwrite("g_gamma", &C3Options::g_gamma)
      .def_readwrite("g_lambda_n", &C3Options::g_lambda_n)
      .def_readwrite("g_lambda_t", &C3Options::g_lambda_t)
      .def_readwrite("g_lambda", &C3Options::g_lambda)
      .def_readwrite("g_u", &C3Options::g_u)
      .def_readwrite("u_vector", &C3Options::u_vector)
      .def_readwrite("u_x", &C3Options::u_x)
      .def_readwrite("u_gamma", &C3Options::u_gamma)
      .def_readwrite("u_lambda_n", &C3Options::u_lambda_n)
      .def_readwrite("u_lambda_t", &C3Options::u_lambda_t)
      .def_readwrite("u_lambda", &C3Options::u_lambda)
      .def_readwrite("u_u", &C3Options::u_u)
      .def_readwrite("mu", &C3Options::mu)
      .def_readwrite("dt", &C3Options::dt)
      .def_readwrite("solve_dt", &C3Options::solve_dt)
      .def_readwrite("num_friction_directions", &C3Options::num_friction_directions)
      .def_readwrite("num_contacts", &C3Options::num_contacts)
      .def_readwrite("Q", &C3Options::Q)
      .def_readwrite("R", &C3Options::R)
      .def_readwrite("G", &C3Options::G)
      .def_readwrite("U", &C3Options::U);
}
}  // namespace pydairlib
}  // namespace dairlib