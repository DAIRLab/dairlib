#include "alip_mpfc_diagram.h"

#include <iostream>
#include "dairlib/lcmt_robot_output.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/footstep_planning/alip_mpfc_system.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "examples/perceptive_locomotion/gains/alip_mpfc_gains.h"

#include "geometry/convex_polygon_set.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
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

using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
using drake::multibody::Frame;
using drake::systems::TriggerType;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerTypeSet;

AlipMPFCDiagram::AlipMPFCDiagram(const std::string& gains_filename,
                                 double debug_publish_period) {

  auto gains_mpc =
      drake::yaml::LoadYamlFile<AlipMpfcGainsImport>(gains_filename);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  auto instance_w_spr = AddCassieMultibody(
      &plant_w_spr,
      nullptr,
      true,
      "examples/Cassie/urdf/cassie_v2.urdf",
      true,
      false
  );

  plant_w_spr.Finalize();
  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  gains_mpc.SetFilterData(
      plant_w_spr.CalcTotalMass(*context_w_spr), gains_mpc.h_des);


  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);

  double single_support_duration = gains_mpc.ss_time;
  double double_support_duration = gains_mpc.ds_time;


  std::vector<int> left_right_fsm_states;
  std::vector<int> post_left_right_fsm_states;
  std::vector<double> state_durations;

  left_right_fsm_states = {left_stance_state, right_stance_state};
  post_left_right_fsm_states = {post_right_double_support_state,
                                post_left_double_support_state};
  state_durations = {single_support_duration, single_support_duration};

  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};

  const auto& planner_solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(
              "examples/perceptive_locomotion/gains/gurobi_options_planner.yaml"
          )).GetAsSolverOptions(drake::solvers::GurobiSolver::id());

  auto foot_placement_controller =
      builder.AddSystem<AlipMPFC>(
          plant_w_spr, context_w_spr.get(), left_right_fsm_states,
          post_left_right_fsm_states, state_durations, double_support_duration,
          left_right_toe, gains_mpc.gains, planner_solver_options);

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);


  // Create the diagram
  builder.BuildInto(this);
  DrawAndSaveDiagramGraph(*this, "../alip_mpfc");

}


} // dairlib
} // perceptive_locomotion