#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/perception_module_diagram.h"
#include "examples/perceptive_locomotion/diagrams/alip_mpfc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/non_render_perception_module_diagram.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_output_from_footstep.h"
#include "examples/perceptive_locomotion/diagrams/rl_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/radio_receiver_module.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_output_from_rl.h"
#include "examples/perceptive_locomotion/diagrams/cassie_realsense_driver_diagram.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using perceptive_locomotion::RLOscDiagram;
using perceptive_locomotion::MpfcOscDiagram;
using perceptive_locomotion::HikingSimDiagram;
using perceptive_locomotion::PerceptionModuleDiagram;
using perceptive_locomotion::NonRenderPerceptionModuleDiagram;
using perceptive_locomotion::MpfcOscDiagramInputType;
using perceptive_locomotion::AlipMPFCDiagram;
using perceptive_locomotion::MpfcOutputFromFootstep;
using perceptive_locomotion::RadioReceiverModule;
using perceptive_locomotion::MpfcOutputFromRL;
using perceptive_locomotion::CassieRealSenseDriverDiagram;

using multibody::SquareSteppingStoneList;

PYBIND11_MODULE(diagrams, m) {
  m.doc() = "Binding perceptive locomotion diagrams for "
            "use in python simulations";

  using py_rvp = py::return_value_policy;

  py::enum_<MpfcOscDiagramInputType>(m, "MpfcOscDiagramInputType")
      .value("kFootstepCommand", MpfcOscDiagramInputType::kFootstepCommand)
      .value("kLcmtAlipMpcOutput", MpfcOscDiagramInputType::kLcmtAlipMpcOutput);

  py::class_<MpfcOscDiagram, drake::systems::Diagram<double>>(
      m, "MpfcOscDiagram")
      .def(py::init<drake::multibody::MultibodyPlant<double>&,
           const std::string&, const std::string&, const std::string&,
           MpfcOscDiagramInputType>(),
           py::arg("plant"),
           py::arg("osc_gains_filename"),
           py::arg("mpc_gains_filename"),
           py::arg("oscp_settings_filename"),
           py::arg("input_type"))
      .def("get_input_port_state",
           &MpfcOscDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_footstep_command",
           &MpfcOscDiagram::get_input_port_footstep_command,
           py_rvp::reference_internal)
      .def("get_input_port_alip_mpc_output",
           &MpfcOscDiagram::get_input_port_alip_mpc_output,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &MpfcOscDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_actuation",
           &MpfcOscDiagram::get_output_port_actuation,
           py_rvp::reference_internal)
      .def("get_output_port_fsm",
           &MpfcOscDiagram::get_output_port_fsm,
           py_rvp::reference_internal)
      .def("get_output_port_alip",
           &MpfcOscDiagram::get_output_port_alip,
           py_rvp::reference_internal)
      .def("get_output_port_switching_time",
           &MpfcOscDiagram::get_output_port_switching_time,
           py_rvp::reference_internal)
      .def("get_output_port_swing_ft_tracking_error",
           &MpfcOscDiagram::get_output_port_swing_ft_tracking_error,
           py_rvp::reference_internal)
      .def("get_output_port_pelvis_yaw",
           &MpfcOscDiagram::get_output_port_pelvis_yaw,
           py_rvp::reference_internal)
      .def("get_plant", &MpfcOscDiagram::get_plant, py_rvp::reference_internal)
      .def("SetSwingFootPositionAtLiftoff",
           &MpfcOscDiagram::SetSwingFootPositionAtLiftoff);

  py::class_<RLOscDiagram, drake::systems::Diagram<double>>(
      m, "RLOscDiagram")
      .def(py::init<drake::multibody::MultibodyPlant<double>&,
           const std::string&, const std::string&, const std::string&>(),
           py::arg("plant"),
           py::arg("osc_gains_filename"),
           py::arg("mpc_gains_filename"),
           py::arg("osqp_settings_filename"))
      .def("get_input_port_state",
           &RLOscDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_rl",
           &RLOscDiagram::get_input_port_rl,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &RLOscDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_actuation",
           &RLOscDiagram::get_output_port_actuation,
           py_rvp::reference_internal)
      .def("get_output_port_fsm",
           &RLOscDiagram::get_output_port_fsm,
           py_rvp::reference_internal)
      .def("get_output_port_alip",
           &RLOscDiagram::get_output_port_alip,
           py_rvp::reference_internal)
      .def("get_output_port_switching_time",
           &RLOscDiagram::get_output_port_switching_time,
           py_rvp::reference_internal)
      .def("get_output_port_swing_ft_tracking_error",
           &RLOscDiagram::get_output_port_swing_ft_tracking_error,
           py_rvp::reference_internal)
      .def("get_output_port_pelvis_yaw",
           &RLOscDiagram::get_output_port_pelvis_yaw,
           py_rvp::reference_internal)
      .def("get_plant", &RLOscDiagram::get_plant, py_rvp::reference_internal)
      .def("SetSwingFootPositionAtLiftoff",
           &RLOscDiagram::SetSwingFootPositionAtLiftoff);

  py::class_<HikingSimDiagram, drake::systems::Diagram<double>>(
      m, "HikingSimDiagram")
      .def(py::init<const std::variant<std::string, SquareSteppingStoneList>&,
                    const std::string&>(),
           py::arg("tarrain_yaml"), py::arg("camera_pose_yaml"))
      .def("get_input_port_actuation",
           &HikingSimDiagram::get_input_port_actuation,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &HikingSimDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_input_port_spatial_force",
           &HikingSimDiagram::get_input_port_spatial_force,
           py_rvp::reference_internal)     
      .def("get_output_port_state_lcm",
           &HikingSimDiagram::get_output_port_state_lcm,
           py_rvp::reference_internal)
      .def("get_output_port_state",
           &HikingSimDiagram::get_output_port_state,
           py_rvp::reference_internal)
      .def("get_output_port_cassie_out",
           &HikingSimDiagram::get_output_port_cassie_out,
           py_rvp::reference_internal)
      .def("get_output_port_lcm_radio",
           &HikingSimDiagram::get_output_port_lcm_radio,
           py_rvp::reference_internal)
      .def("get_output_port_depth_image",
           &HikingSimDiagram::get_output_port_depth_image,
           py_rvp::reference_internal)
      .def("get_plant",
           &HikingSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_depth_camera_info",
           &HikingSimDiagram::get_depth_camera_info)
      .def("AddDrakeVisualizer",
           &HikingSimDiagram::AddDrakeVisualizer,
           py_rvp::reference_internal)
      .def("SetPlantInitialConditionFromIK",
           &HikingSimDiagram::SetPlantInitialConditionFromIK)
      .def("SetPlantInitialCondition",
           &HikingSimDiagram::SetPlantInitialCondition);


  py::class_<PerceptionModuleDiagram, drake::systems::Diagram<double>>(
      m, "PerceptionModuleDiagram")
      .def(py::init<std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
           std::string,
           std::map<std::string, drake::systems::sensors::CameraInfo>,
           std::string>(),
           py::arg("plant"), py::arg("elevation_mapping_params_yaml_path"),
           py::arg("depth_sensor_info"), py::arg("joint_offsets_yaml"))
      .def("get_input_port_cassie_out",
           &PerceptionModuleDiagram::get_input_port_cassie_out,
           py_rvp::reference_internal)
      .def("get_input_port_depth_image",
           &PerceptionModuleDiagram::get_input_port_depth_image,
           py_rvp::reference_internal)
      .def("get_output_port_state",
           &PerceptionModuleDiagram::get_output_port_state,
           py_rvp::reference_internal)
      .def("get_output_port_robot_output",
           &PerceptionModuleDiagram::get_output_port_robot_output,
           py_rvp::reference_internal)
      .def("get_output_port_elevation_map",
           &PerceptionModuleDiagram::get_output_port_elevation_map,
           py_rvp::reference_internal)
      .def("InitializeEkf",
           py::overload_cast<drake::systems::Context<double>*,
              const Eigen::VectorXd&, const Eigen::VectorXd&>(
                  &PerceptionModuleDiagram::InitializeEkf, py::const_))
      .def("InitializeElevationMap", &PerceptionModuleDiagram::InitializeElevationMap)
      .def("Make", &PerceptionModuleDiagram::Make);

  py::class_<NonRenderPerceptionModuleDiagram, drake::systems::Diagram<double>>(
      m, "NonRenderPerceptionModuleDiagram")
      .def(py::init<std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
           std::string,
           std::map<std::string, drake::systems::sensors::CameraInfo>,
           geometry::ConvexPolygonSet,
           std::string>(),
           py::arg("plant"), py::arg("elevation_mapping_params_yaml_path"),
           py::arg("depth_sensor_info"), py::arg("polygon_terain"),
           py::arg("joint_offsets_yaml"))
      .def("get_input_port_cassie_out",
           &NonRenderPerceptionModuleDiagram::get_input_port_cassie_out,
           py_rvp::reference_internal)
      .def("get_input_port_gt_state",
           &NonRenderPerceptionModuleDiagram::get_input_port_gt_state,
           py_rvp::reference_internal)
      .def("get_output_port_state",
           &NonRenderPerceptionModuleDiagram::get_output_port_state,
           py_rvp::reference_internal)
      .def("get_output_port_robot_output",
           &NonRenderPerceptionModuleDiagram::get_output_port_robot_output,
           py_rvp::reference_internal)
      .def("get_output_port_elevation_map",
           &NonRenderPerceptionModuleDiagram::get_output_port_elevation_map,
           py_rvp::reference_internal)
      .def("InitializeEkf",
           py::overload_cast<drake::systems::Context<double>*,
              const Eigen::VectorXd&, const Eigen::VectorXd&>(
                  &NonRenderPerceptionModuleDiagram::InitializeEkf, py::const_))
      .def("InitializeElevationMap", &NonRenderPerceptionModuleDiagram::InitializeElevationMap)
      .def("Make", &NonRenderPerceptionModuleDiagram::Make);

  py::class_<AlipMPFCDiagram, drake::systems::Diagram<double>>(
      m, "AlipMPFCDiagram")
      .def(py::init<const drake::multibody::MultibodyPlant<double>&,
                    const std::string&, double>(),
                    py::arg("plant"), py::arg("gains_filename"),
                    py::arg("debug_publish_period"))
      .def("get_input_port_state",
           &AlipMPFCDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_footholds",
           &AlipMPFCDiagram::get_input_port_footholds,
           py_rvp::reference_internal)
      .def("get_input_port_vdes",
           &AlipMPFCDiagram::get_input_port_vdes,
           py_rvp::reference_internal)
      .def("get_output_port_mpc_output",
           &AlipMPFCDiagram::get_output_port_mpc_output,
           py_rvp::reference_internal);

  py::class_<RadioReceiverModule, drake::systems::Diagram<double>>(
      m, "RadioReceiverModule")
      .def(py::init<const std::string&, drake::lcm::DrakeLcmInterface*>(),
          py::arg("cassie_out_channel"), py::arg("lcm"));

  py::class_<MpfcOutputFromFootstep, drake::systems::LeafSystem<double>>(
      m, "MpfcOutputFromFootstep")
      .def(py::init<double, double, const drake::multibody::MultibodyPlant<double>&>(),
          py::arg("T_ss"), py::arg("T_ds"), py::arg("plant"))
      .def("get_input_port_state", &MpfcOutputFromFootstep::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_footstep", &MpfcOutputFromFootstep::get_input_port_footstep,
           py_rvp::reference_internal);

  py::class_<MpfcOutputFromRL, drake::systems::LeafSystem<double>>(
      m, "MpfcOutputFromRL")
      .def(py::init<>());

  py::class_<CassieRealSenseDriverDiagram, drake::systems::Diagram<double>>(
        m, "CassieRealSenseDriverDiagram")
        .def(py::init<const std::string&>(), py::arg("points_channel"))
        .def("InitializeElevationMap", &CassieRealSenseDriverDiagram::InitializeElevationMap)
        .def("lcm", &CassieRealSenseDriverDiagram::lcm, py_rvp::reference_internal)
        .def("plant", &CassieRealSenseDriverDiagram::plant, py_rvp::reference_internal)
        .def("get_input_port_state", &CassieRealSenseDriverDiagram::get_input_port_state, py_rvp::reference_internal)
        .def("get_input_port_contact", &CassieRealSenseDriverDiagram::get_input_port_contact, py_rvp::reference_internal)
        .def("start_rs", &CassieRealSenseDriverDiagram::start_rs)
        .def("stop_rs", &CassieRealSenseDriverDiagram::stop_rs);
  }


}
}