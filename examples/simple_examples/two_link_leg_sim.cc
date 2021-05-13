/* @file
A four bar linkage demo demonstrating the use of a linear bushing as
a way to model a kinematic loop. It shows:
  - How to model a four bar linkage in SDF.
  - Use the `multibody::Parser` to load a model from an SDF file into a
    MultibodyPlant.
  - Model a revolute joint with a `multibody::LinearBushingRollPitchYaw` to
    model a closed kinematic chain.

  Refer to README.md for more details.
*/
#include <drake/multibody/tree/linear_spring_damper.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
//#include "drake/systems/analysis/simulator_gflags.h"
#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Vector3d;

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::LinearBushingRollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
namespace dairlib {
namespace examples {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");
DEFINE_double(simulator_realtime_rate, 1.0, "Realtime rate");

DEFINE_double(
    force_stiffness, 30000,
    "Force (translational) stiffness value for kx, ky, kz in N/m of the "
    "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(
    force_damping, 1500,
    "Force (translational) damping value for dx, dy, dz in N·s/m of the "
    "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(torque_stiffness, 30000,
              "Torque (rotational) stiffness value for k₀, k₁, k₂ in N·m/rad "
              "of the LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(
    torque_damping, 1500,
    "Torque (rotational) damping value for d₀, d₁, and d₂ in N·m·s/rad of "
    "the LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(applied_torque, 0.0,
              "Constant torque applied to joint_WA, denoted Tᴀ in the README.");

DEFINE_double(
    initial_velocity, 3.0,
    "Initial velocity, q̇A, of joint_WA. Default set to 3 radians/second ≈ "
    "171.88 degrees/second so that the model has some motion.");
DEFINE_double(
    dt, 1e-5,
    "Initial velocity, q̇A, of joint_WA. Default set to 3 radians/second ≈ "
    "171.88 degrees/second so that the model has some motion.");

int do_main() {
  // Build a generic MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_dt));

  // Make and add the four_bar model from an SDF model.
  const std::string relative_name =
      "examples/simple_examples/two_link_leg.urdf";
  const std::string full_name = dairlib::FindResourceOrThrow(relative_name);

  Parser parser(&plant);
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());

  // Get the two frames that define the bushing, namely frame Bc that is
  // welded to the end of link B and frame Cb that is welded to the end of
  // link C. Although the bushing allows 6 degrees-of-freedom between frames
  // Bc and Cb, the stiffness and damping constants are chosen to
  //  approximate
  // the connection between frames Bc and Cb as having only one rotational
  // degree of freedom along the bushing's z-axis.
  const Frame<double>& bc_bushing = plant.GetFrameByName("lower_bushing");
  const Frame<double>& cb_bushing = plant.GetFrameByName("bottom");

  // See the README for a discussion of how these parameters were selected
  // for this particular example.
  const double k_xyz = FLAGS_force_stiffness;
  const double d_xyz = FLAGS_force_damping;
  const double k_012 = FLAGS_torque_stiffness;
  const double d_012 = FLAGS_torque_damping;

  // See the documentation for LinearBushingRollPitchYaw.
  // This particular choice of parameters models a z-axis revolute joint.
  // Note: since each link is constrained to rigid motion in the world X-Z
  // plane (X-Y plane of the bushing) by the revolute joints specified in
  //  the SDF, it is unnecessary for the bushing to have non-zero values
  //    for:
  //  k_z, d_z, k_0, d_0, k_1, d_1. They are left here as an example of how
  //    to
  // parameterize a general z-axis revolute joint without other constraints.
  const Vector3d force_stiffness_constants{k_xyz, k_xyz, k_xyz};  // N/m
  const Vector3d force_damping_constants{d_xyz, d_xyz, d_xyz};    // N·s/m
  const Vector3d torque_stiffness_constants{k_012, 0, k_012};     // N·m/rad
  const Vector3d torque_damping_constants{d_012, 0, d_012};  //    N·m·s/rad

  // Add a bushing force element where the joint between link B and link C
  // should be in an ideal 4-bar linkage.
  plant.AddForceElement<LinearBushingRollPitchYaw>(
      bc_bushing, cb_bushing, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);
  //  plant.AddForceElement<drake::multibody::LinearSpringDamper>(
  //      plant.GetBodyByName("lower_bushing"), Eigen::VectorXd::Zero(3),
  //      plant.GetBodyByName("lower_bushing"), Eigen::VectorXd::Zero(3),
  //      1e-3, 1e6, 2e3);
  // Add Ground
  multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);  // Add ground
  // We are done defining the model. Finalize and build the diagram.
  plant.Finalize();

  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the four bar system.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& four_bar_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // A constant source for applied torque (Tᴀ) at joint_WA.
  plant.get_actuation_input_port().FixValue(&four_bar_context,
                                            FLAGS_applied_torque);

  //  // Set initial conditions so the model will have some motion
  const PrismaticJoint<double>& height =
      plant.GetJointByName<PrismaticJoint>("upper_rail");
  const PrismaticJoint<double>& lower_height =
      plant.GetJointByName<PrismaticJoint>("lower_rail");
  const RevoluteJoint<double>& elbow_joint =
      plant.GetJointByName<RevoluteJoint>("elbow_joint");
  const RevoluteJoint<double>& upper_joint =
      plant.GetJointByName<RevoluteJoint>("upper_joint");
  //  const RevoluteJoint<double>& elbow_joint =
  //      four_bar.GetJointByName<RevoluteJoint>("elbow_joint");
  //  const RevoluteJoint<double>& joint_WC =
  //      four_bar.GetJointByName<RevoluteJoint>("joint_WC");
  //  const RevoluteJoint<double>& joint_AB =
  //      four_bar.GetJointByName<RevoluteJoint>("joint_AB");
  //
  //  // See the README for an explanation of these angles.
  //  const double qA = atan2(sqrt(15.0), 1.0);  // about 75.52°
  //  const double qB = M_PI - qA;               // about 104.48°
  //  const double qC = qB;                      // about 104.48°
  //
  //  height.set_translation(&four_bar_context, 2.0);
  //  elbow_joint.set_angle(&four_bar_context, M_PI / 2);
  //  upper_joint.set_angle(&four_bar_context, -M_PI / 4);
  //  joint_AB.set_angle(&four_bar_context, qB);
  //  joint_WC.set_angle(&four_bar_context, qC);
  //
  //  // Set q̇A,the rate of change in radians/second of the angle qA.
  //  joint_WA.set_angular_rate(&four_bar_context, FLAGS_initial_velocity);

  std::cout << "upper_rail_idx: " << height.index() << std::endl;
  std::cout << "lower_rail_idx: " << lower_height.index() << std::endl;
  std::cout << "elbow_joint_idx: " << elbow_joint.index() << std::endl;
  std::cout << "upper_joint_idx: " << upper_joint.index() << std::endl;
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  Eigen::VectorXd x0 =
      Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  x0(0) = 1.5;
  x0(3) = M_PI / 2;
  double beta = 0.5 * (M_PI - x0(3));
  x0(2) = -beta;
  x0(1) = x0(0) - sin(x0(3))/(sin(beta));
  x0(plant.num_positions() + 3) = -0.5;
  plant.SetPositionsAndVelocities(&plant_context, x0);
  plant.get_actuation_input_port().FixValue(&plant_context,
                                               FLAGS_applied_torque);
  // Create a simulator and run the simulation
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  //  simulator.set_target_realtime_rate(FLAGS_simulator_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);

  //  // Print some useful statistics
  //  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating the use of a linear bushing as "
      "a way to model a kinematic loop. Launch drake-visualizer before running "
      "this example.");
  // Changes the default realtime rate to 1.0, so the visualization looks
  // realistic. Otherwise, it finishes so fast that we can't appreciate the
  // motion. Users can still change it on command-line, e.g. "
  // --simulator_target_realtime_rate=0.5" to slow it down.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::examples::FLAGS_simulator_realtime_rate = 1.0;
  return dairlib::examples::do_main();
}
