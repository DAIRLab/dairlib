#include <iostream>

#include "alip_mpfc_diagram.h"
#include "dairlib/lcmt_robot_output.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/footstep_planning/alip_mpfc_system.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "geometry/convex_polygon_set.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {
namespace perceptive_locomotion {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;

using systems::controllers::AlipMPFC;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::AlipMPFCGains;

using drake::systems::TriggerType;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;

AlipMPFCDiagram::AlipMPFCDiagram(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& gains_filename,
    double debug_publish_period) :
    lcm_local("udpm://239.255.76.67:7667?ttl=0"),
    plant_(plant),
    left_toe(LeftToeFront(plant_)),
    left_heel(LeftToeRear(plant_)),
    left_toe_mid({(left_toe.first + left_heel.first) / 2,
                  plant.GetFrameByName("toe_left")}),
    right_toe_mid({(left_toe.first + left_heel.first) / 2,
                   plant.GetFrameByName("toe_right")}) {

  left_right_toe.push_back(left_toe_mid);
  left_right_toe.push_back(right_toe_mid);

  gains_mpc = drake::yaml::LoadYamlFile<AlipMpfcGainsImport>(gains_filename);
  plant_context_ = plant_.CreateDefaultContext();

  gains_mpc.SetFilterData(
      plant_.CalcTotalMass(*plant_context_), gains_mpc.h_des);


  // Build the controller diagram
  DiagramBuilder<double> builder;

  double single_support_duration = gains_mpc.ss_time;
  double double_support_duration = gains_mpc.ds_time;

  state_durations = {single_support_duration, single_support_duration};

  planner_solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(
              "examples/perceptive_locomotion/gains/gurobi_options_planner.yaml"
          )).GetAsSolverOptions(drake::solvers::GurobiSolver::id());

  auto foot_placement_controller = builder.AddSystem<AlipMPFC>(
      plant_, plant_context_.get(), left_right_fsm_states,
      post_left_right_fsm_states, state_durations, double_support_duration,
      left_right_toe, gains_mpc.gains, planner_solver_options);

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_);

  builder.Connect(state_receiver->get_output_port(),
                  foot_placement_controller->get_input_port_state());

  if (debug_publish_period > 0) {
    auto mpc_debug_pub = builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_mpc_debug>(
            "ALIP_MPFC_DEBUG", &lcm_local, debug_publish_period));
    builder.Connect(foot_placement_controller->get_output_port_mpc_debug(),
                    mpc_debug_pub->get_input_port());
  }

  input_port_state_ = builder.ExportInput(
      state_receiver->get_input_port(), "lcmt_robot_output"
  );
  input_port_footholds_ = builder.ExportInput(
      foot_placement_controller->get_input_port_footholds(), "footholds"
  );
  input_port_vdes_ = builder.ExportInput(
      foot_placement_controller->get_input_port_vdes(), "desired_velocity"
  );
  output_port_mpc_output_ = builder.ExportOutput(
      foot_placement_controller->get_output_port_mpc_output(),
      "lcmt_alip_mpc_output"
  );

  // Create the diagram
  builder.BuildInto(this);
  DrawAndSaveDiagramGraph(*this, "../alip_mpfc_diagram");

}


} // dairlib
} // perceptive_locomotion