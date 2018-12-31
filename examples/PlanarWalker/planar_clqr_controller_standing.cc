#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
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
#include "systems/controllers/clqr_controller_planar.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/solve_multibody_constraints.h"
#include "planar_utils.h"
#include "planar_solver.h"

using std::cout;
using std::endl;
using std::vector;

using Eigen::VectorXi;
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

using dairlib::multibody::SolveTreeConstraints;
using dairlib::multibody::CheckTreeConstraints;
using dairlib::multibody::SolveFixedPointConstraints;
using dairlib::multibody::CheckFixedPointConstraints;
using dairlib::multibody::SolveTreeAndFixedPointConstraints;
using dairlib::multibody::CheckTreeAndFixedPointConstraints;
using dairlib::multibody::SolveFixedPointFeasibilityConstraints;
using dairlib::systems::AffineParams;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::OutputVector;
using dairlib::PlanarJointsWithinLimits;
using dairlib::PlanarPlant;
using dairlib::SolvePlanarStandingConstraints;


namespace dairlib{


// Simulation parameters.
DEFINE_double(realtime_rate, 1, "Simulator realtime rate");
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.9, "The static coefficient of friction");
DEFINE_double(ud, 0.9, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "timestepping", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");


//Class to serve as a connecer between the OutputVector type input port of the clqr controller and a BasicVector port through which the plant states are sent
class InfoConnector: public LeafSystem<double> {

  public:

    InfoConnector(int num_positions, int num_velocities, int num_efforts):
      num_states_(num_positions + num_velocities), num_efforts_(num_efforts) {

      this->DeclareVectorInputPort(BasicVector<double>(
            num_positions + num_velocities + num_efforts + 3 + 1));
      this->DeclareVectorOutputPort(OutputVector<double>(
            num_positions, num_velocities, num_efforts), &dairlib::InfoConnector::CopyOut);
    }

  private:

    const int num_states_;
    const int num_efforts_;

    void CopyOut(const Context<double>& context, OutputVector<double>* output) const {

      const auto info = this->EvalVectorInput(context, 0);
      const VectorX<double> info_vec = info->get_value();
      output->SetState(info_vec.head(num_states_));
      output->set_timestamp(0);
    }

};



int do_main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  auto tree_autodiff = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalker.urdf", 
      drake::multibody::joints::kFixed, tree.get());

  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalker.urdf", 
      drake::multibody::joints::kFixed, tree_autodiff.get());

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
  drake::multibody::AddFlatTerrainToWorld(tree_autodiff.get(), terrain_size, terrain_depth);
  
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
  RigidBodyPlant<AutoDiffXd> plant_autodiff(std::move(tree_autodiff), FLAGS_dt);


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
  std::map<std::string, int>  map = plant->get_rigid_body_tree().computePositionNameToIndexMap();

  for(auto elem: map)
  {
      cout << elem.first << " " << elem.second << endl;
  }


  x0(map.at("planar_x")) = 0.0;
  x0(map.at("planar_z")) = 2;
  x0(map.at("hip_pin")) = 0.1;
  x0(map.at("left_knee_pin")) = 0.0;
  x0(map.at("right_knee_pin")) = 0.0;


  // Making sure tha the joints are within limits
  DRAKE_DEMAND(PlanarJointsWithinLimits(plant->get_rigid_body_tree(), x0));

  std::vector<int> fixed_joints;

 
  // **********************************************************************************
  
  
  const int num_tree_constraints = 0;
  const int num_contacts = 2;
  const int num_constraints_per_contact = 2;
  const int num_contact_constraints = num_contacts * num_constraints_per_contact;
  const int num_total_constraints = num_tree_constraints + num_contact_constraints;
  VectorXd q_init = x0.head(num_positions);
  VectorXd u_init = VectorXd::Zero(num_efforts);
  VectorXd lambda_init = VectorXd::Zero(num_total_constraints);
  const bool print_debug = false;
  std::cout << num_total_constraints << std::endl;


  // **********************************************************************************

  cout << "Solving" << endl;
  vector<VectorXd> q_u_l_sol = SolvePlanarFixedPointAndStandingConstraints(*plant,
                                                                               plant_autodiff,
                                                                               num_total_constraints,
                                                                               q_init,
                                                                               u_init,
                                                                               lambda_init,
                                                                               fixed_joints,
                                                                               print_debug); 


  //VectorXd qsol =  SolvePlanarStandingConstraints(plant->get_rigid_body_tree(), 
  //                                        q_init, 
  //                                        fixed_joints,
  //                                        print_debug);
  //std::cout << "Solution: " << qsol.transpose() << std::endl;
  //
  //x0.head(num_positions) = qsol;
    

  VectorXd q_sol = q_u_l_sol.at(0);
  VectorXd v_sol = VectorXd::Zero(num_velocities);
  VectorXd x_sol(num_states);
  x_sol << q_sol, v_sol;
  VectorXd u_sol = q_u_l_sol.at(1);
  VectorXd lambda_sol = q_u_l_sol.at(2);
  x0.head(num_positions) = q_sol;

  KinematicsCache<double> k_cache_sol = plant->get_rigid_body_tree().doKinematics(q_sol, v_sol);
  MatrixXd M = plant->get_rigid_body_tree().massMatrix(k_cache_sol);

  std::cout << "Eigen Values of M: " << endl << M.eigenvalues() << endl;

  //// Making sure that the joint state solution is within limits
  DRAKE_DEMAND(PlanarJointsWithinLimits(plant->get_rigid_body_tree(), x_sol));

  cout << "***************** q sol *****************" << endl;
  cout << q_sol.transpose() << endl;
  cout << "***************** u sol *****************" << endl;
  cout << u_sol.transpose() << endl;
  cout << "***************** lambda sol *****************" << endl;
  cout << lambda_sol.transpose() << endl;

  PlanarPlant<double> planar_plant(*plant);
  cout << "***************** Mvdot final *******************" << endl;
  cout << planar_plant.CalcMVdotPlanarStanding(x_sol, 
                                               u_sol, 
                                               lambda_sol) << endl;


  //Parameter matrices for LQR
  MatrixXd Q = MatrixXd::Identity(num_states - 2*num_total_constraints, num_states - 2*num_total_constraints);
  //Q corresponding to the positions
  MatrixXd Q_p = MatrixXd::Identity(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints)*100;
  //Q corresponding to the velocities
  MatrixXd Q_v = MatrixXd::Identity(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints)*1.0;
  Q.block(0, 0, Q_p.rows(), Q_p.cols()) = Q_p;
  Q.block(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints, Q_v.rows(), Q_v.cols()) = Q_v;
  MatrixXd R = MatrixXd::Identity(num_efforts, num_efforts)*1;
  
  const double fixed_point_tolerance = 1e-3;

  //Building the controller
  auto clqr_controller = builder.AddSystem<systems::ClqrControllerPlanar>(*plant,
                                                                    plant_autodiff,
                                                                    x_sol,
                                                                    u_sol,
                                                                    lambda_sol,
                                                                    Q,
                                                                    R, 
                                                                    fixed_point_tolerance);
  

  
  VectorXd K_vec = clqr_controller->GetKVec();
  //K_vec = K_vec*0.0;
  VectorXd E = u_sol; 
  //E = E*0.0;
  VectorXd x_desired = x_sol;
  cout << "----------------------------------------------------------------------------------------" << endl;
  cout << "K: " << K_vec.transpose() << endl;
  cout << "E: " << E.transpose() << endl;
  cout << "xdes: " << x_desired.transpose() << endl;

  vector<int> input_info_sizes{num_states, num_efforts, 3, 1};
  auto info_connector = builder.AddSystem<InfoConnector>(
      num_positions, num_velocities, num_efforts);
  auto multiplexer_info = builder.AddSystem<Multiplexer<double>>(
      input_info_sizes);

  auto constant_zero_source_efforts = builder.AddSystem<ConstantVectorSource<double>>(
      VectorXd::Zero(num_efforts));
  auto constant_zero_source_imu = builder.AddSystem<ConstantVectorSource<double>>(
      VectorXd::Zero(3));
  auto constant_zero_source_timestamp = builder.AddSystem<ConstantVectorSource<double>>(
      VectorXd::Zero(1));


  VectorXd params_vec(num_states*num_efforts + num_efforts + num_states);
  params_vec << K_vec, E, x_desired;
  AffineParams params(num_states, num_efforts);
  params.SetDataVector(params_vec);

  auto constant_params_source = builder.AddSystem<ConstantVectorSource<double>>(params);
  auto control_output = builder.AddSystem<SubvectorPassThrough<double>>(
          (clqr_controller->get_output_port(0)).size(), 0, (clqr_controller->get_output_port(0)).size() - 1);

 
  builder.Connect(plant->state_output_port(), multiplexer_info->get_input_port(0));
  builder.Connect(constant_zero_source_efforts->get_output_port(), multiplexer_info->get_input_port(1));
  builder.Connect(constant_zero_source_imu->get_output_port(), multiplexer_info->get_input_port(2));
  builder.Connect(constant_zero_source_timestamp->get_output_port(), multiplexer_info->get_input_port(3));
  builder.Connect(multiplexer_info->get_output_port(0), info_connector->get_input_port(0));
  builder.Connect(info_connector->get_output_port(0), clqr_controller->get_input_port_info());
  builder.Connect(constant_params_source->get_output_port(), clqr_controller->get_input_port_params());
  builder.Connect(clqr_controller->get_output_port(0), control_output->get_input_port());
  builder.Connect(control_output->get_output_port(), plant->actuator_command_input_port()); 

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
    diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());

  if (FLAGS_simulation_type != "timestepping") {

    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
    state.SetFromVector(x_sol);
  } else {

    drake::systems::BasicVector<double>& state = context.get_mutable_discrete_state(0);
    state.SetFromVector(x_sol); 

  }


  //context.FixInputPort(0, MatrixXd::Zero(num_efforts, 1));

  
  //simulator.set_publish_every_time_step(false);
  //simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();
  
  lcm.StartReceiveThread();
  
  simulator.StepTo(std::numeric_limits<double>::infinity());
  //simulator.StepTo(0.000000001);
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
