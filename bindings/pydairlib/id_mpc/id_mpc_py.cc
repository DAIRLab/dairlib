#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "systems/controllers/id_mpc/costs/mpc_reference.h"
#include "systems/controllers/id_mpc/core/constrained_inverse_dynamics_info.h"
#include "examples/id_mpc/cassie_mpc_utils.h"

#include "drake/bindings/pydrake/common/value_pybind.h"

namespace py = pybind11;

namespace dairlib::systems::controllers::id_mpc {

PYBIND11_MODULE(id_mpc, m) {
    m.doc() = "Python bindings for IDMPC classes"; // Optional module docstring

  py::module::import("pydrake.trajectories");
  py::module::import("pydrake.systems.framework");

  py::class_<MPCReference>(m, "MPCReference")
    .def(py::init<>())
    // Expose member variables as properties
    .def_readwrite("q_traj", &MPCReference::q_traj_)
    .def_readwrite("quat_traj", &MPCReference::quat_traj_)
    .def_readwrite("v_traj", &MPCReference::v_traj_)
    .def_readwrite("lambda_traj", &MPCReference::lambda_traj_)
    .def_readwrite("u_traj", &MPCReference::u_traj_)
    .def_readwrite("task_space_trajs", &MPCReference::task_space_trajs_)
    .def_readwrite("knot_times", &MPCReference::knot_times_)
    .def_readwrite("active_contacts", &MPCReference::active_contacts_)
    .def_readwrite("touchdown_ee_names", &MPCReference::touchdown_ee_names_)
    .def_readwrite("touchdown_ee_points", &MPCReference::touchdown_ee_points_)
    // Expose member functions
    .def("AppendContactsToKnot", &MPCReference::AppendContactsToKnot,
         py::arg("i"),
         py::arg("contacts"),
         "Append contacts to a specific knot point");

  drake::pydrake::AddValueInstantiation<MPCReference>(m);

  py::class_<ConstrainedDynamicsInfo>(m, "ConstrainedDynamicsInfo")
    .def(py::init<std::string>(), py::arg("urdf"))

    // Read-only properties
    .def("nq", &ConstrainedDynamicsInfo::nq)
    .def("nv", &ConstrainedDynamicsInfo::nv)
    .def("nu", &ConstrainedDynamicsInfo::nu)
    .def("nh", &ConstrainedDynamicsInfo::nh)
    .def("nc", &ConstrainedDynamicsInfo::nc)
    .def("nlambda", &ConstrainedDynamicsInfo::nlambda)
    .def("nc_active", &ConstrainedDynamicsInfo::nc_active)
    .def("nx", &ConstrainedDynamicsInfo::nx)
    .def("n_constraint_total", &ConstrainedDynamicsInfo::n_constraint_total)
    .def("variable_count", &ConstrainedDynamicsInfo::variable_count)

    // Plant access methods
    .def("get_plant", &ConstrainedDynamicsInfo::get_plant, py::return_value_policy::reference)
    .def("get_mutable_plant", &ConstrainedDynamicsInfo::get_mutable_plant, py::return_value_policy::reference)
    .def("get_plant_ad", &ConstrainedDynamicsInfo::get_plant_ad, py::return_value_policy::reference)

    // Vector selection methods
    .def("select_contact_force_from_lambda",
        &ConstrainedDynamicsInfo::select_contact_force_from_lambda<double>,
        py::arg("name"), py::arg("lambda"))
    .def("get_lambda_for_active_contacts",
        &ConstrainedDynamicsInfo::get_lambda_for_active_contacts<double>,
        py::arg("contacts"), py::arg("lambda"))

    // Contact and constraint methods
    .def("contacts", &ConstrainedDynamicsInfo::contacts)
    .def("finalize", &ConstrainedDynamicsInfo::Finalize)
    .def("AddDistanceConstraint", &ConstrainedDynamicsInfo::AddDistanceConstraint,
        py::arg("body_A"), py::arg("pt_A"),
        py::arg("body_B"), py::arg("pt_B"),
        py::arg("distance"))
    .def("AddContactPoint", &ConstrainedDynamicsInfo::AddContactPoint,
        py::arg("name"), py::arg("body"),
        py::arg("point_in_body_frame"),
        py::arg("active_constraint_directions"),
        py::arg("friction_coefficient"))

    // Kinematics and dynamics methods
    .def("MakeContext", &ConstrainedDynamicsInfo::MakeContext<double>)
    .def("set_plant_state_if_new",
        &ConstrainedDynamicsInfo::SetPlantStateIfNew<double>,
        py::arg("x"), py::arg("context"))
    .def("make_empty_kinematics_results",
        &ConstrainedDynamicsInfo::MakeEmptyKinematicsResults<double>)
    .def("evaluate_kinematics",
        &ConstrainedDynamicsInfo::EvaluateKinematics<double>,
        py::arg("context"), py::arg("active_contacts"))
    .def("evaluate_inverse_dynamics",
        &ConstrainedDynamicsInfo::EvaluateInverseDynamics<double>,
        py::arg("context"), py::arg("kinematics"),
        py::arg("vdot"), py::arg("lambda"))
    .def("estimate_constraint_forces_for_fixed_point",
        &ConstrainedDynamicsInfo::EstimateConstraintForcesForFixedPoint,
        py::arg("context"), py::arg("u"),
        py::arg("active_contacts"));

  // Define KinematicsResults struct
  py::class_<ConstrainedDynamicsInfo::KinematicsResults<double>>(m, "KinematicsResults")
    .def(py::init<>())
    .def_readwrite("J", &ConstrainedDynamicsInfo::KinematicsResults<double>::J)
    .def_readwrite("c", &ConstrainedDynamicsInfo::KinematicsResults<double>::c)
    .def_readwrite("cdot", &ConstrainedDynamicsInfo::KinematicsResults<double>::cdot)
    .def_readwrite("qdot", &ConstrainedDynamicsInfo::KinematicsResults<double>::qdot);

  m.def("MakeCassieGaitParams", &MakeCassieGaitParams)
   .def("MakeCassieDynamics", &MakeCassieDynamics);


  py::class_<IDMPCParams>(m, "IDMPCParams")
      .def(py::init<>())
      .def_readwrite("N", &IDMPCParams::N)
      .def_readwrite("dt", &IDMPCParams::dt)
      .def_readwrite("num_full_torque_knots", &IDMPCParams::num_full_torque_knots)
      .def_readwrite("num_intervals_between_impacts", &IDMPCParams::num_intervals_between_impacts)
      .def_readwrite("mu", &IDMPCParams::mu)
      .def_property("Wq",
                    [](const IDMPCParams& p) { return p.Wq; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wq = val; })
      .def_property("Wrot",
                    [](const IDMPCParams& p) { return p.Wrot; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wrot = val; })
      .def_property("Wv",
                    [](const IDMPCParams& p) { return p.Wv; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wv = val; })
      .def_property("Wlambda",
                    [](const IDMPCParams& p) { return p.Wlambda; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wlambda = val; })
      .def_property("Wu",
                    [](const IDMPCParams& p) { return p.Wu; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wu = val; })
      .def_property("Wq_final",
                    [](const IDMPCParams& p) { return p.Wq_final; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wq_final = val; })
      .def_property("Wrot_final",
                    [](const IDMPCParams& p) { return p.Wrot_final; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wrot_final = val; })
      .def_property("Wv_final",
                    [](const IDMPCParams& p) { return p.Wv_final; },
                    [](IDMPCParams& p, const Eigen::MatrixXd& val) { p.Wv_final = val; })
      .def("__repr__",
           [](const IDMPCParams& p) {
             return "IDMPCParams(N=" + std::to_string(p.N) +
                 ", dt=" + std::to_string(p.dt) +
                 ", num_full_torque_knots=" + std::to_string(p.num_full_torque_knots) +
                 ", num_intervals_between_impacts=" + std::to_string(p.num_intervals_between_impacts) +
                 ", mu=" + std::to_string(p.mu) + ")";
           });

  // Bind the YAML loading function
  m.def("LoadIDMPCParamsFromYaml", &LoadIDMPCParamsFromYaml,
        py::arg("filename"),
        "Load IDMPC parameters from a YAML file");

  // Add convenience function to create diagonal weight matrices
  m.def("create_diagonal_weight_matrix",
        [](const std::vector<double>& diagonal) {
          return Eigen::Map<const Eigen::VectorXd>(
              diagonal.data(), diagonal.size()).asDiagonal();
        },
        "Create a diagonal matrix from a vector of weights");

}

}
