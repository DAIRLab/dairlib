#include <iostream>

#include "cassie_mpfc_diagram.h"

#include "dairlib/lcmt_alip_s2s_mpfc_debug.hpp"
#include "dairlib/lcmt_robot_output.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/controllers/footstep_planning/cf_mpfc_system.h"

#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "geometry/convex_polygon_set.h"

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

using systems::controllers::Alips2sMPFCSystem;
using systems::controllers::CFMPFCSystem;
using systems::controllers::alip_utils::PointOnFramed;

using drake::systems::TriggerType;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;

template <mpfc MPC>
CassieMPFCDiagram<MPC>::CassieMPFCDiagram(
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

  plant_context_ = plant_.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  auto foot_placement_controller = builder.AddSystem<MPC>(
      plant_, plant_context_.get(), left_right_fsm_states,
      post_left_right_fsm_states, left_right_toe, gains_filename,
      "examples/perceptive_locomotion/gains/gurobi_options_planner.yaml");

  foot_placement_controller->MakeDrivenByStandaloneSimulator(0.01);

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_);

  builder.Connect(state_receiver->get_output_port(),
                  foot_placement_controller->get_input_port_state());

  input_port_state_ = builder.ExportInput(
      state_receiver->get_input_port(), "lcmt_robot_output"
  );

  if (debug_publish_period > 0) {
    auto mpc_debug_pub = builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_alip_s2s_mpfc_debug>(
            "ALIP_MPFC_DEBUG", &lcm_local, debug_publish_period));
    builder.Connect(foot_placement_controller->get_output_port_mpc_debug(),
                    mpc_debug_pub->get_input_port());
    auto state_debug_pub = builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_robot_output>(
            "CASSIE_STATE_MPC_DEBUG", &lcm_local, debug_publish_period));
    builder.ConnectInput(input_port_state_, state_debug_pub->get_input_port());
  }


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
  output_port_mpfc_debug_ = builder.ExportOutput(
      foot_placement_controller->get_output_port_mpc_debug(),
      "lcmt_alip_s2s_mpfc_debug"
  );

  // Create the diagram
  builder.BuildInto(this);
  DrawAndSaveDiagramGraph(*this, "../alip_mpfc_diagram");

}

template class CassieMPFCDiagram<Alips2sMPFCSystem>;
template class CassieMPFCDiagram<CFMPFCSystem>;

} // dairlib
} // perceptive_locomotion