#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/clqr_controller.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/solve_multibody_constraints.h"
#include "cassie_utils.h"
#include "cassie_solver.h"

using std::cout;
using std::endl;
using std::vector;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::EigenSolver;
using drake::VectorX;
using drake::MatrixX;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;
using drake::systems::DrakeVisualizer;
using drake::systems::Multiplexer;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 1e-2,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");



namespace dairlib{

int do_main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;
  std::unique_ptr<RigidBodyTree<double>> tree = makeFloatingBaseCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf");

  //Floating base adds 6 additional states for the base position and orientation
  const int num_positions = tree->get_num_positions();
  const int num_velocities = tree->get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_efforts = tree->get_num_actuators();
  const int num_constraints = tree->getNumPositionConstraints();
  
  cout << "Number of actuators: " << num_efforts << endl;
  cout << "Number of generalized coordinates: " << num_positions << endl;
  cout << "Number of generalized velocities: " << num_velocities << endl;
  cout << "Number of tree constraints: " << num_constraints << endl;

  const double terrain_size = 4;
  const double terrain_depth = 0.05;

  drake::multibody::AddFlatTerrainToWorld(tree.get(), terrain_size, terrain_depth);
  
  cout << "---------------------------------------------------------------------" << endl;

  for(int i=0; i<tree->get_num_bodies(); i++)
  {
    cout << tree->get_body(i).get_name() << " " << i << endl;
  }

  cout << "---------------------------------------------------------------------" << endl;
  
  drake::systems::DiagramBuilder<double> builder;

  if (FLAGS_simulation_type != "timestepping")
    FLAGS_dt = 0.0;
  
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);

  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);

  // Visualizer
  DrakeVisualizer& visualizer_publisher = 
    *builder.template AddSystem<DrakeVisualizer>(
        plant->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  builder.Connect(plant->state_output_port(), 
                  visualizer_publisher.get_input_port(0));


  VectorXd x0 = VectorXd::Zero(num_states);
  x0(0) = 0;
  x0(1) = 0;
  x0(2) = -0.0131423;
  x0(3) = 1.42392;
  x0(4) = (M_PI/2)*1;
  x0(5) = 1.86435e-13;
  x0(6) = 0.0479853;
  x0(7) = 0.262;
  x0(8) = 0.393;
  x0(9) = -0.393;
  x0(10) = 1.07425;
  x0(11) = 1.07465;
  x0(12) = -2.83473;
  x0(13) = -2.8353;
  x0(14) = 0.0127599;
  x0(15) = 0.0129346;
  x0(16) = 3;
  x0(17) = 3;
  x0(18) = 0.0213757;
  x0(19) = 0.021549;
  x0(20) = -0.554888;
  x0(21) = -0.554827;

  VectorXd q0 = x0.head(num_positions);
  VectorXd v0 = x0.tail(num_velocities);
  std::map<std::string, int>  map = plant->get_rigid_body_tree().computePositionNameToIndexMap();

  for(auto elem: map)
  {
      cout << elem.first << " " << elem.second << endl;
  }



  // Making sure tha the joints are within limits
  DRAKE_DEMAND(CassieJointsWithinLimits(plant->get_rigid_body_tree(), x0));

  KinematicsCache<double> k_cache = plant->get_rigid_body_tree().doKinematics(q0, v0);
  MatrixXd M = plant->get_rigid_body_tree().massMatrix(k_cache);

  std::cout << "Eigen Values of M: " << endl << M.eigenvalues() << endl;

  //auto diagram = builder.Build();

  //drake::systems::Simulator<double> simulator(*diagram);
  //drake::systems::Context<double>& context =
  //  diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());

  //if (FLAGS_simulation_type != "timestepping") {
  //  drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
  //  state.SetFromVector(x0);
  //}
  //
  //simulator.set_target_realtime_rate(1.0);
  //simulator.Initialize();
  //
  //lcm.StartReceiveThread();
  //
  ////simulator.StepTo(std::numeric_limits<double>::infinity());
  //simulator.StepTo(0.000000001);
  
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}


