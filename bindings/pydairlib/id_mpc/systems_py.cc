#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "systems/controllers/id_mpc/walking/walking_reference_system.h"

namespace py = pybind11;

namespace dairlib::systems::controllers::id_mpc {

PYBIND11_MODULE(systems, m) {
    m.doc() = "Python bindings for IDMPC related systems";

  // Bind GaitParams struct
  py::class_<GaitParams>(m, "GaitParams")
    .def(py::init<>())
    .def_readwrite("t_ss", &GaitParams::t_ss)
    .def_readwrite("t_ds", &GaitParams::t_ds)
    .def_readwrite("mpc_dt", &GaitParams::mpc_dt)
    .def_readwrite("mpc_N", &GaitParams::mpc_N)
    .def_readwrite("stance_width", &GaitParams::stance_width)
    .def_readwrite("standing_pose_q", &GaitParams::standing_pose_q)
    .def_readwrite("standing_pose_lambda", &GaitParams::standing_pose_lambda)
    .def_readwrite("standing_pose_u", &GaitParams::standing_pose_u)
    .def_readwrite("left_foot_body_name", &GaitParams::left_foot_body_name)
    .def_readwrite("right_foot_body_name", &GaitParams::right_foot_body_name)
    .def_readwrite("floating_base_name", &GaitParams::floating_base_name)
    .def_readwrite("left_foot_contacts", &GaitParams::left_foot_contacts)
    .def_readwrite("right_foot_contacts", &GaitParams::right_foot_contacts)
    .def_readwrite("left_leg_actuator_idxs", &GaitParams::left_leg_actuator_idxs)
    .def_readwrite("right_leg_actuator_idxs", &GaitParams::right_leg_actuator_idxs)
    .def_readwrite("left_leg_holonomic_constraint_idxs", &GaitParams::left_leg_holonomic_constraint_idxs)
    .def_readwrite("right_leg_holonomic_constraint_idxs", &GaitParams::right_leg_holonomic_constraint_idxs)
    .def_readwrite("foot_midpoint", &GaitParams::foot_midpoint);

    // Bind FSM state enum
  py::enum_<fsm_info::fsm_state>(m, "FSMState")
    .value("LEFT", fsm_info::fsm_state::kLeft)
    .value("POST_LEFT_DOUBLE", fsm_info::fsm_state::kPostLeftDouble)
    .value("RIGHT", fsm_info::fsm_state::kRight)
    .value("POST_RIGHT_DOUBLE", fsm_info::fsm_state::kPostRightDouble)
    .export_values();

  // Bind fsm_info struct
  py::class_<fsm_info>(m, "FSMInfo")
    .def(py::init<>())
    .def_readwrite("state", &fsm_info::state)
    .def_readwrite("prev_switch_time", &fsm_info::prev_switch_time)
    .def_readwrite("next_switch_time", &fsm_info::next_switch_time)
    .def("is_double_stance", static_cast<bool (fsm_info::*)() const>(&fsm_info::is_double_stance))
    .def_static("next_fsm", &fsm_info::next_fsm)
    .def_static("is_double_stance_static", static_cast<bool (*)(fsm_info::fsm_state)>(&fsm_info::is_double_stance),
    "Check if the given state is a double stance state");


  // Bind WalkingReferenceSystem
  py::class_<WalkingReferenceSystem, drake::systems::LeafSystem<double>>(m, "WalkingReferenceSystem")
    .def(py::init<const ConstrainedDynamicsInfo&,
                 drake::systems::Context<double>*,
                 const GaitParams&>(),
         py::arg("dynamics"),
         py::arg("plant_context"),
         py::arg("params"))
    .def("get_input_port_state", &WalkingReferenceSystem::get_input_port_state,
         py::return_value_policy::reference_internal)
    .def("get_input_port_vdes", &WalkingReferenceSystem::get_input_port_vdes,
         py::return_value_policy::reference_internal)
    .def("add_swing_foot_traj_cost_to_mpc", &WalkingReferenceSystem::AddSwingFootTrajCostToMPC,
         py::arg("mpc"),
         py::arg("Q"));


}
}