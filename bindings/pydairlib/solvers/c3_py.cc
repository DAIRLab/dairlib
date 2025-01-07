#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "solvers/c3.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using dairlib::solvers::C3;
using dairlib::solvers::LCS;

class PyC3 : public C3 {
 public:
  /* Inherit the constructors */
  using C3::C3;

  /* Trampoline (need one for each virtual function) */
  Eigen::VectorXd SolveSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const int admm_iteration, const int& warm_start_index) override {
    PYBIND11_OVERRIDE_PURE(Eigen::VectorXd,       /* Return type */
                           C3,                    /* Parent class */
                           SolveSingleProjection, /* Name of function in C++
                                                     (must match Python name) */
                           U, delta_c, E, F, H, c, admm_iteration,
                           warm_start_index /* Argument(s) */
    );
  };
};

PYBIND11_MODULE(c3, m) {
  py::class_<C3::CostMatrices>(m, "CostMatrices")
      .def(py::init<const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&>(),
           py::arg("Q"), py::arg("R"), py::arg("G"), py::arg("U"))
      .def_readwrite("Q", &C3::CostMatrices::Q)
      .def_readwrite("R", &C3::CostMatrices::R)
      .def_readwrite("G", &C3::CostMatrices::G)
      .def_readwrite("U", &C3::CostMatrices::U);

  py::class_<C3, PyC3>(m, "C3")
      .def(py::init<const LCS&, const C3::CostMatrices&,
                    const std::vector<Eigen::VectorXd>&, const C3Options&>(),
           py::arg("LCS"), py::arg("costs"), py::arg("x_desired"),
           py::arg("options"))

      .def("Solve", &C3::Solve, py::arg("x0"))
      .def("UpdateLCS", &C3::UpdateLCS, py::arg("lcs"))
      .def("UpdateTarget", &C3::UpdateTarget, py::arg("x_des"))
      .def("AddLinearConstraint", &C3::AddLinearConstraint, py::arg("A"),
           py::arg("lower_bound"), py::arg("upper_bound"),
           py::arg("constraint"))
      .def("RemoveConstraints", &C3::RemoveConstraints)
      .def("ADMMStep", &C3::ADMMStep, py::arg("x0"), py::arg("delta"),
           py::arg("w"), py::arg("G"), py::arg("admm_iteration"))
      .def("SolveQP", &C3::SolveQP, py::arg("x0"), py::arg("G"), py::arg("WD"),
           py::arg("admm_iteration"), py::arg("is_final_solve") = false)
      .def("SolveProjection", &C3::SolveProjection, py::arg("G"), py::arg("WZ"),
           py::arg("admm_iteration"))
      .def("SolveSingleProjection", &C3::SolveSingleProjection, py::arg("U"),
           py::arg("delta_c"), py::arg("E"), py::arg("F"), py::arg("H"),
           py::arg("c"), py::arg("admm_iteration"), py::arg("warm_start_index"))
      .def("SetOsqpSolverOptions", &C3::SetOsqpSolverOptions,
           py::arg("options"))
      .def("GetFullSolution", &C3::GetFullSolution)
      .def("GetStateSolution", &C3::GetStateSolution)
      .def("GetForceSolution", &C3::GetForceSolution)
      .def("GetInputSolution", &C3::GetInputSolution)
      .def("GetDualDeltaSolution", &C3::GetDualDeltaSolution)
      .def("GetDualWSolution", &C3::GetDualWSolution);

  py::class_<LCS>(m, "LCS")
      .def(py::init<const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::VectorXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::VectorXd>&, double>(),
           py::arg("A"), py::arg("B"), py::arg("D"), py::arg("d"), py::arg("E"),
           py::arg("F"), py::arg("H"), py::arg("c"), py::arg("dt"))

      .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXd&,
                    const Eigen::MatrixXd&, const Eigen::VectorXd&,
                    const Eigen::MatrixXd&, const Eigen::MatrixXd&,
                    const Eigen::MatrixXd&, const Eigen::VectorXd&, const int&,
                    double>(),
           py::arg("A"), py::arg("B"), py::arg("D"), py::arg("d"), py::arg("E"),
           py::arg("F"), py::arg("H"), py::arg("c"), py::arg("N"),
           py::arg("dt"))

      .def(py::init<const LCS&>(), py::arg("other"))

      .def("simulate", &LCS::Simulate, py::arg("x_init"), py::arg("input"),
           "Simulate the system for one step")

      .def_readwrite("A_", &LCS::A_)
      .def_readwrite("B_", &LCS::B_)
      .def_readwrite("D_", &LCS::D_)
      .def_readwrite("d_", &LCS::d_)
      .def_readwrite("E_", &LCS::E_)
      .def_readwrite("F_", &LCS::F_)
      .def_readwrite("H_", &LCS::H_)
      .def_readwrite("c_", &LCS::c_)
      .def_readwrite("W_x_", &LCS::W_x_)
      .def_readwrite("W_l_", &LCS::W_l_)
      .def_readwrite("W_u_", &LCS::W_u_)
      .def_readwrite("w_", &LCS::w_)
      .def_readwrite("has_tangent_linearization_",
                     &LCS::has_tangent_linearization_)
      .def_readwrite("J_c_", &LCS::J_c_)
      .def_readwrite("N_", &LCS::N_)
      .def_readwrite("dt_", &LCS::dt_)
      .def_readwrite("n_", &LCS::n_)
      .def_readwrite("m_", &LCS::m_)
      .def_readwrite("k_", &LCS::k_);

  py::class_<C3Options>(m, "C3Options")
      .def(py::init<>())
      .def_readwrite("admm_iter", &C3Options::admm_iter)
      .def_readwrite("rho", &C3Options::rho)
      .def_readwrite("rho_scale", &C3Options::rho_scale)
      .def_readwrite("num_threads", &C3Options::num_threads)
      .def_readwrite("delta_option", &C3Options::delta_option)
      .def_readwrite("projection_type", &C3Options::projection_type)
      .def_readwrite("contact_model", &C3Options::contact_model)
      .def_readwrite("M", &C3Options::M)
      .def_readwrite("warm_start", &C3Options::warm_start)
      .def_readwrite("use_predicted_x0", &C3Options::use_predicted_x0)
      .def_readwrite("end_on_qp_step", &C3Options::end_on_qp_step)
      .def_readwrite("use_robust_formulation",
                     &C3Options::use_robust_formulation)
      .def_readwrite("solve_time_filter_alpha",
                     &C3Options::solve_time_filter_alpha)
      .def_readwrite("publish_frequency", &C3Options::publish_frequency)
      .def_readwrite("u_horizontal_limits", &C3Options::u_horizontal_limits)
      .def_readwrite("u_vertical_limits", &C3Options::u_vertical_limits)
      .def_readwrite("workspace_limits", &C3Options::workspace_limits)
      .def_readwrite("workspace_margins", &C3Options::workspace_margins)
      .def_readwrite("N", &C3Options::N)
      .def_readwrite("gamma", &C3Options::gamma)
      .def_readwrite("mu", &C3Options::mu)
      .def_readwrite("dt", &C3Options::dt)
      .def_readwrite("solve_dt", &C3Options::solve_dt)
      .def_readwrite("num_friction_directions",
                     &C3Options::num_friction_directions)
      .def_readwrite("num_contacts", &C3Options::num_contacts)
      .def_readwrite("Q", &C3Options::Q)
      .def_readwrite("R", &C3Options::R)
      .def_readwrite("G", &C3Options::G)
      .def_readwrite("U", &C3Options::U)
      .def_readwrite("w_Q", &C3Options::w_Q)
      .def_readwrite("w_R", &C3Options::w_R)
      .def_readwrite("w_G", &C3Options::w_G)
      .def_readwrite("w_U", &C3Options::w_U)
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
      .def_readwrite("u_u", &C3Options::u_u);
}
}  // namespace pydairlib
}  // namespace dairlib