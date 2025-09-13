#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/sampling_c3/push_anything_solver_benchmarker.h"

namespace py = pybind11;
using dairlib::PushAnythingSolverBenchmarker;

PYBIND11_MODULE(push_anything_solver_benchmarker, m) {
  py::class_<PushAnythingSolverBenchmarker>(m, "PushAnythingSolverBenchmarker")
      .def(py::init<bool>(), py::arg("verbose") = false)
      .def("solve", &PushAnythingSolverBenchmarker::Solve,
           py::arg("x_lcs_curr"), py::arg("x_lcs_des"),
           "Solve the pushing problem given current and desired LCS states.")
      .def("get_qp_solve_times",
           &PushAnythingSolverBenchmarker::GetQPSolveTimes,
           "Get recorded QP solve times.")
      .def("get_projection_solve_times",
           &PushAnythingSolverBenchmarker::GetProjectionSolveTimes,
           "Get recorded projection solve times.");
}
