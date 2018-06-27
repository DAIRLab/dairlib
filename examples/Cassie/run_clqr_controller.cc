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

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/clqr_controller.h"
#include "systems/framework/output_vector.h"
#include "multibody/solve_multibody_constraints.h"

using std::cout;
using std::endl;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using drake::systems::CompliantContactModel;

namespace dairlib{

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");

int do_main(int argc, char* argv[]) 
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    
    drake::lcm::DrakeLcm lcm;
    std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();
    
    const int NUM_EFFORTS = tree->get_num_actuators();
    const int NUM_POSITIONS = tree->get_num_positions();
    const int NUM_VELOCITIES = tree->get_num_velocities();
    const int NUM_STATES = NUM_POSITIONS + NUM_VELOCITIES;
    
    cout << "Number of actuators: " << NUM_EFFORTS << endl;
    cout << "Number of generalized coordinates: " << NUM_POSITIONS << endl;
    cout << "Number of generalized velocities: " << NUM_VELOCITIES << endl;

    KinematicsCache<double> kcache = tree->doKinematics(tree->getZeroConfiguration());
    cout << tree->positionConstraints(kcache) << endl;
    cout << tree->positionConstraintsJacobian(kcache) << endl;
    cout << "--------------------------------------------------------" << endl;
    
    
    drake::systems::DiagramBuilder<double> builder;
    
    auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree));
    
      // Note: this sets identical contact parameters across all object pairs:
    
    drake::systems::CompliantMaterial default_material;
    default_material.set_youngs_modulus(FLAGS_youngs_modulus)
        .set_dissipation(FLAGS_dissipation)
        .set_friction(FLAGS_us, FLAGS_ud);
    plant->set_default_compliant_material(default_material);
    drake::systems::CompliantContactModelParameters model_parameters;
    model_parameters.characteristic_radius = FLAGS_contact_radius;
    model_parameters.v_stiction_tolerance = FLAGS_v_tol;
    plant->set_contact_model_parameters(model_parameters);



    //auto constant_zero_source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
            //drake::VectorX<double>::Zero(plant->actuator_command_input_port().size()));

    // The vector source is connected to the inputs of the rigid body tree (Joints of the robot)
    //builder.Connect(constant_zero_source->get_output_port(), plant->actuator_command_input_port());

    // Adding the visualizer to the diagram
    drake::systems::DrakeVisualizer& visualizer_publisher =
        *builder.template AddSystem<drake::systems::DrakeVisualizer>(
            plant->get_rigid_body_tree(), &lcm);
    visualizer_publisher.set_name("visualizer_publisher");
    builder.Connect(plant->state_output_port(),
                            visualizer_publisher.get_input_port(0));

    VectorXd x0 = VectorXd::Zero(NUM_POSITIONS + NUM_VELOCITIES);
    std::map<std::string, int>  map = plant->get_rigid_body_tree().computePositionNameToIndexMap();

    x0(map.at("hip_pitch_left")) = .269;
    x0(map.at("hip_pitch_right")) = .269;
    // x0(map.at("achilles_hip_pitch_left")) = -.44;
    // x0(map.at("achilles_hip_pitch_right")) = -.44;
    // x0(map.at("achilles_heel_pitch_left")) = -.105;
    // x0(map.at("achilles_heel_pitch_right")) = -.105;
    x0(map.at("knee_left")) = -.644;
    x0(map.at("knee_right")) = -.644;
    x0(map.at("ankle_joint_left")) = .792;
    x0(map.at("ankle_joint_right")) = .792;
    
    // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
    // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;
    
    // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
    // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;
    
    x0(map.at("toe_left")) = -60.0*M_PI/180.0;
    x0(map.at("toe_right")) = -60.0*M_PI/180.0;

    std::vector<int> fixed_joints;
    fixed_joints.push_back(map.at("hip_pitch_left"));
    fixed_joints.push_back(map.at("hip_pitch_right"));
    fixed_joints.push_back(map.at("knee_left"));
    fixed_joints.push_back(map.at("knee_right"));
    

 //   auto q0 = solvePositionConstraints(plant->get_rigid_body_tree(),
 //       x0.head(NUM_POSITIONS), fixed_joints);
 //   x0.head(NUM_POSITIONS) = q0;

    VectorXd x_init = x0;
    VectorXd xu_init = VectorXd::Zero(NUM_STATES + NUM_EFFORTS);
    xu_init.head(NUM_STATES) = x0;


    //std::cout << plant->get_rigid_body_tree().B << std::endl;

    Matrix<double, Dynamic, Dynamic> Q = MatrixXd::Identity(NUM_STATES, NUM_STATES);
    Matrix<double, Dynamic, Dynamic> R = MatrixXd::Identity(NUM_EFFORTS, NUM_EFFORTS);

    cout << "Starting" << endl;
    multibody::SolveMultibodyConstraints solver(plant); 
    cout << "Solving" << endl;
    VectorXd xu_sol = solver.solveFP(xu_init);
    VectorXd x_sol = solver.solveTP(x_init);
    VectorXd xu_sol_tpfp = solver.solveTPFP(xu_init);
    cout << "Solved" << endl;

    cout << xu_sol.transpose() << endl;
    cout << "--------------------------------------------------------------------------------------------------------" << endl; 
    cout << x_sol.transpose() << endl;
    cout << "--------------------------------------------------------------------------------------------------------" << endl; 
    cout << xu_sol_tpfp.transpose() << endl;
    cout << "--------------------------------------------------------------------------------------------------------" << endl; 


    //auto clqr_controller = builder.AddSystem<systems::ClqrController>(plant, xu_sol, xd, NUM_POSITIONS, NUM_VELOCITIES, NUM_EFFORTS, Q, R);
    //cout << clqr_controller->get_input_port_output().size() << endl;
    //cout << clqr_controller->get_output_port(0).size() << endl;
    //OutputVector<double> input_port_output_vector(NUM_POSITIONS, NUM_VELOCITIES, NUM_EFFORTS);
    //builder.Connect(plant->state_output_port(), clqr_controller->get_input_port_output());

    //builder.Connect(clqr_controller->get_output_port(0), plant->actuator_command_input_port());

    auto diagram = builder.Build();
    
    drake::systems::Simulator<double> simulator(*diagram);
    drake::systems::Context<double>& context =
        diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
    
    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
    state.SetFromVector(x0);
    
    auto zero_input = Eigen::MatrixXd::Zero(NUM_EFFORTS,1);
    context.FixInputPort(0, zero_input);
    
    //simulator.set_publish_every_time_step(false);
    //simulator.set_publish_at_initialization(false);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    
    lcm.StartReceiveThread();
    
    simulator.StepTo(std::numeric_limits<double>::infinity());
    return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
