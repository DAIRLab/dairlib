#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <drake/bindings/pydrake/common/sorted_pair_pybind.h>

#include "solvers/c3_miqp.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"

namespace py = pybind11;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using py::arg;
using std::vector;

using dairlib::solvers::C3MIQP;
using dairlib::solvers::LCS;

namespace c3 {
namespace pyc3 {

PYBIND11_MODULE(c3, m) {
  py::class_<dairlib::solvers::LCS>(m, "LCS")
      .def(py::init<const vector<MatrixXd>&, const vector<MatrixXd>&,
                   const vector<MatrixXd>&, const vector<VectorXd>&,
                   const vector<MatrixXd>&, const vector<MatrixXd>&,
                   const vector<MatrixXd>&, const vector<VectorXd>&, double>(),
          arg("A"), arg("B"), arg("D"), arg("d"), arg("E"), arg("F"), arg("H"),
          arg("c"), arg("dt"))
      .def(py::init<const MatrixXd&, const MatrixXd&, const MatrixXd&,
                    const MatrixXd&, const MatrixXd&, const MatrixXd&,
                    const MatrixXd&, const VectorXd&, int, double>(),
           arg("A"), arg("B"), arg("D"), arg("d"), arg("E"), arg("F"), arg("H"),
           arg("c"), arg("N"), arg("dt"))
      .def_readwrite("A_", &LCS::A_)
      .def_readwrite("B_", &LCS::B_)
      .def_readwrite("D_", &LCS::D_)
      .def_readwrite("d_", &LCS::d_)
      .def_readwrite("E_", &LCS::E_)
      .def_readwrite("F_", &LCS::F_)
      .def_readwrite("H_", &LCS::H_)
      .def_readwrite("c_", &LCS::c_)
      .def("Simulate", &LCS::Simulate);

  m.def("LinearizePlantToLCS",
        &dairlib::solvers::LCSFactory::LinearizePlantToLCS,
        py::arg("plant"),
        py::arg("context"),
        py::arg("plant_ad"),
        py::arg("context_ad"),
        py::arg("contact_geoms"),
        py::arg("num_friction_directions"),
        py::arg("mu"),
        py::arg("dt"),
        py::arg("N"),
        py::arg("contact_model"));

  {
    using Enum = dairlib::solvers::ContactModel;
    py::enum_<Enum> enum_py(m, "ContactModel");
    enum_py  // BR
        .value("kStewartAndTrinkle", Enum::kStewartAndTrinkle)
        .value("kAnitescu", Enum::kAnitescu);
  }

  py::class_<dairlib::solvers::C3::CostMatrices>(m, "CostMatrices")
      .def(py::init<const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
                    const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&>(),
           arg("Q"), arg("R"), arg("G"), arg("U"));

  py::class_<dairlib::solvers::C3MIQP>(m, "C3MIQP")
      .def(py::init<const LCS&, const dairlib::solvers::C3::CostMatrices&,
                    const vector<VectorXd>&, const C3Options&>(),
           arg("LCS"), arg("costs"), arg("x_des"), arg("c3_options"))
      .def("Solve", &C3MIQP::Solve, arg("x0"))
      .def("UpdateTarget", &C3MIQP::UpdateTarget, arg("x0"))
      .def("UpdateLCS", &C3MIQP::UpdateLCS, arg("lcs"))
      .def("ADMMStep", &C3MIQP::ADMMStep, arg("x0"), arg("delta"), arg("w"),
           arg("G"), arg("admm_iteration"))
      .def("SolveQP", &C3MIQP::SolveQP, arg("x0"), arg("G"), arg("WD"),
           arg("admm_iteration"), arg("is_final_solve"))
      .def("SolveProjection", &C3MIQP::SolveProjection, arg("U"), arg("WZ"), arg("admm_iteration"))
      .def("AddLinearConstraint", &C3MIQP::AddLinearConstraint, arg("A"),
           arg("lower_bound"), arg("upper_bound"), arg("constraint"))
      .def("RemoveConstraints", &C3MIQP::RemoveConstraints)
      .def("GetFullSolution", &C3MIQP::GetFullSolution)
      .def("GetStateSolution", &C3MIQP::GetStateSolution)
      .def("GetForceSolution", &C3MIQP::GetForceSolution)
      .def("GetInputSolution", &C3MIQP::GetInputSolution)
      .def("GetDualDeltaSolution", &C3MIQP::GetDualDeltaSolution)
      .def("GetDualWSolution", &C3MIQP::GetDualWSolution);

  py::class_<C3Options> options(m, "C3Options");
  options.def(py::init<>())
      .def_readwrite("admm_iter", &C3Options::admm_iter)
      .def_readwrite("rho", &C3Options::rho)
      .def_readwrite("rho_scale", &C3Options::rho_scale)
      .def_readwrite("num_threads", &C3Options::num_threads)
      .def_readwrite("delta_option", &C3Options::delta_option)
      .def_readwrite("contact_model", &C3Options::contact_model)
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
      .def_readwrite("U", &C3Options::U);
}

}  // namespace pyc3
}  // namespace c3
