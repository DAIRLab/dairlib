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
#include "systems/controllers/clqr_controller.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/solve_multibody_constraints.h"
#include "cassie_utils.h"
#include "cassie_solver.h"

using std::cout;
using std::endl;
using std::vector;

using Eigen::VectorXi;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using drake::VectorX;
using drake::MatrixX;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;
using drake::systems::Multiplexer;

using dairlib::SolveCassieStandingConstraints;
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


namespace dairlib{


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

//Class to serve as a selective pass through to convert the floating base states to the fixed base states needed by the controller
class FloatToFixedConnector: public LeafSystem<double> {

  public:
    FloatToFixedConnector(int num_positions, int num_velocities, int num_efforts, int num_extra):
      num_positions_(num_positions), num_velocities_(num_velocities), num_efforts_(num_efforts), num_extra_(num_extra) {

    this->DeclareVectorInputPort(BasicVector<double>(num_positions_ + num_velocities_ + num_efforts_ + 3 + 1));
    this->DeclareVectorOutputPort(BasicVector<double>(num_positions_ + num_velocities_ + num_efforts_ + 3 + 1 - 2*num_extra_), &dairlib::FloatToFixedConnector::CopyOut);
  }


  private:
    
    int num_positions_;
    int num_velocities_;
    int num_efforts_;
    int num_extra_;

    void CopyOut(const Context<double>& context, BasicVector<double>* output) const {

      const auto input = this->EvalVectorInput(context, 0);
      const VectorX<double> input_vector = input->get_value();
      VectorX<double> output_vector(input_vector.size() - 2*num_extra_);
      output_vector << input_vector.segment(num_extra_, num_positions_ - num_extra_),
        input_vector.segment(num_positions_ + num_extra_, num_velocities_ - num_extra_),
        input_vector.segment(num_positions_ + num_velocities_, num_efforts_ + 3 + 1);

      output->SetFromVector(output_vector);

    }
};

//Class to serve as a pass through port that just prints the value of the input port. Useful for debugging
class DebugPassThrough: public LeafSystem<double> {

  public:
    DebugPassThrough(int size, bool debug_flag = true): size_(size), debug_flag_(debug_flag) {

    this->DeclareVectorInputPort(BasicVector<double>(size_));
    this->DeclareVectorOutputPort(BasicVector<double>(size_), &dairlib::DebugPassThrough::CopyOut);
  }

  const int size_;
  bool debug_flag_;

  private:
    void CopyOut(const Context<double>& context, BasicVector<double>* output) const {

      const auto input = this->EvalVectorInput(context, 0);
      const VectorX<double> input_vector = input->get_value();


      if(debug_flag_) {
        cout << input_vector.transpose() << endl;
        cout << "------------------------------------------------------------------------------------------" << endl;
      }
      output->SetFromVector(input_vector);
    }
};

//Function to take the state of a floating base model and extract the state for the fixed base model by removing the 
//positions and velocities corresponding to the base
VectorXd ExtractFixedStateFromFloating(VectorXd x_float, int num_positions_float, int num_velocities_float, int num_extra_positions, int num_extra_velocities)
{
  const int num_positions_fixed = num_positions_float - num_extra_positions;
  const int num_velocities_fixed = num_velocities_float - num_extra_velocities;
  const int fixed_size = num_positions_fixed + num_velocities_fixed;

  VectorXd x_fixed(fixed_size);
  x_fixed.head(num_positions_fixed) = x_float.segment(6, num_positions_fixed);
  x_fixed.tail(num_velocities_fixed) = x_float.segment(num_positions_float + 6, num_velocities_fixed);

  return x_fixed;
}

VectorXd ComputeUAnalytical(const RigidBodyTree<double>& tree, VectorXd x) {

  MatrixXd B = tree.B;
  auto k_cache = tree.doKinematics(x.head(tree.get_num_positions()), x.tail(tree.get_num_velocities()));
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd C = tree.dynamicsBiasTerm(k_cache, no_external_wrenches, true);
  VectorXd u = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(C);

  return u;

}




int do_main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;
  std::unique_ptr<RigidBodyTree<double>> tree = makeFloatingBaseCassieTreePointer();
  std::unique_ptr<RigidBodyTree<double>> tree_autodiff = makeFloatingBaseCassieTreePointer();

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
  
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree));
  RigidBodyPlant<AutoDiffXd> plant_autodiff(std::move(tree_autodiff));


  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);


  // Adding the visualizer to the diagram
  drake::systems::DrakeVisualizer& visualizer_publisher =
      *builder.template AddSystem<drake::systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  //builder.Connect(plant->state_output_port(),
                          //visualizer_publisher.get_input_port(0));

  auto debug_pass_through = builder.AddSystem<DebugPassThrough>(num_states, false);
  builder.Connect(plant->state_output_port(), debug_pass_through->get_input_port(0));
  builder.Connect(debug_pass_through->get_output_port(0), visualizer_publisher.get_input_port(0));


  VectorXd x0 = VectorXd::Zero(num_states);
  std::map<std::string, int>  map_sim = plant->get_rigid_body_tree().computePositionNameToIndexMap();
  std::map<std::string, int>  map_model = plant->get_rigid_body_tree().computePositionNameToIndexMap();

  for(auto elem: map_sim)
  {
      cout << elem.first << " " << elem.second << endl;
  }


  x0(map_sim.at("base_x")) = 0.0;
  x0(map_sim.at("base_y")) = 0.0;
  x0(map_sim.at("base_z")) = 2.2;

  x0(map_sim.at("hip_roll_left")) = 0.1;
  x0(map_sim.at("hip_roll_right")) = -0.1;
  x0(map_sim.at("hip_yaw_left")) = 0;
  x0(map_sim.at("hip_yaw_right")) = 0;
  x0(map_sim.at("hip_pitch_left")) = .269;
  x0(map_sim.at("hip_pitch_right")) = .269;
  // x0(map.at("achilles_hip_pitch_left")) = -.44;
  // x0(map.at("achilles_hip_pitch_right")) = -.44;
  // x0(map.at("achilles_heel_pitch_left")) = -.105;
  // x0(map.at("achilles_heel_pitch_right")) = -.105;
  x0(map_sim.at("knee_left")) = -.744;
  x0(map_sim.at("knee_right")) = -.744;
  x0(map_sim.at("ankle_joint_left")) = .81;
  x0(map_sim.at("ankle_joint_right")) = .81;
  
  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;
  
  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;
  
  x0(map_sim.at("toe_left")) = -60*M_PI/180.0;
  x0(map_sim.at("toe_right")) = -60*M_PI/180.0;

  VectorXd x_init = x0;

  // Making sure tha the joints are within limits
  DRAKE_DEMAND(CassieJointsWithinLimits(plant->get_rigid_body_tree(), x_init));

  std::vector<int> fixed_joints;

  //fixed_joints.push_back(map_model.at("base_roll"));
  //fixed_joints.push_back(map_model.at("base_pitch"));
  fixed_joints.push_back(map_model.at("base_yaw"));

  fixed_joints.push_back(map_model.at("hip_roll_left"));
  fixed_joints.push_back(map_model.at("hip_roll_right"));
  fixed_joints.push_back(map_model.at("hip_yaw_left"));
  fixed_joints.push_back(map_model.at("hip_yaw_right"));
  //fixed_joints.push_back(map_model.at("hip_pitch_left"));
  //fixed_joints.push_back(map_model.at("hip_pitch_right"));
  fixed_joints.push_back(map_model.at("knee_left"));
  fixed_joints.push_back(map_model.at("knee_right"));
  //fixed_joints.push_back(map_model.at("ankle_joint_left"));
  //fixed_joints.push_back(map_model.at("ankle_joint_right"));
  //fixed_joints.push_back(map_model.at("toe_left"));
  //fixed_joints.push_back(map_model.at("toe_right"));

  
  const int num_tree_constraints = 2;
  const int num_contacts = 4;
  const int num_constraints_per_contact = 3;
  const int num_contact_constraints = num_contacts * num_constraints_per_contact;
  const int num_total_constraints = num_tree_constraints + num_contact_constraints;
  VectorXd q_init = x0.head(num_positions);
  VectorXd u_init = VectorXd::Zero(num_efforts);
  VectorXd lambda_init = VectorXd::Zero(num_total_constraints);
  const bool print_debug = false;

  vector<VectorXd> q_u_l_sol = SolveCassieTreeFixedPointAndStandingConstraints(*plant,
                                                                               plant_autodiff,
                                                                               num_total_constraints,
                                                                               q_init,
                                                                               u_init,
                                                                               lambda_init,
                                                                               fixed_joints,
                                                                               print_debug); 

  VectorXd q_sol = q_u_l_sol.at(0);
  VectorXd v_sol = VectorXd::Zero(num_velocities);
  VectorXd x_sol(num_states);
  x_sol << q_sol, v_sol;
  VectorXd u_sol = q_u_l_sol.at(1);
  VectorXd lambda_sol = q_u_l_sol.at(2);

  cout << "***************** q sol *****************" << endl;
  cout << q_sol.transpose() << endl;
  cout << "***************** u sol *****************" << endl;
  cout << u_sol.transpose() << endl;
  cout << "***************** lambda sol *****************" << endl;
  cout << lambda_sol.transpose() << endl;

  CassiePlant<double> cassie_plant(*plant);
  cout << "***************** Mvdot final *******************" << endl;
  cout << cassie_plant.CalcMVdotCassieStanding(x_sol, 
                                               u_sol, 
                                               lambda_sol) << endl;
  

  //Parameter matrices for LQR
  MatrixXd Q = MatrixXd::Identity(num_states - 2*num_total_constraints, num_states - 2*num_total_constraints);
  //Q corresponding to the positions
  MatrixXd Q_p = MatrixXd::Identity(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints)*10.0;
  //Q corresponding to the velocities
  MatrixXd Q_v = MatrixXd::Identity(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints)*10.0;
  Q.block(0, 0, Q_p.rows(), Q_p.cols()) = Q_p;
  Q.block(num_states/2 - num_total_constraints, num_states/2 - num_total_constraints, Q_v.rows(), Q_v.cols()) = Q_v;
  MatrixXd R = MatrixXd::Identity(num_efforts, num_efforts)*10;


  //Building the controller
  //auto clqr_controller = builder.AddSystem<systems::ClqrController>(*plant,
  //                                                                  plant_autodiff,
  //                                                                  x_sol,
  //                                                                  u_sol,
  //                                                                  lambda_sol,
  //                                                                  Q,
  //                                                                  R);
  //VectorXd K_vec = clqr_controller->GetKVec();
  //VectorXd C = u_sol; 
  //VectorXd x_desired = x_sol;
  //cout << "----------------------------------------------------------------------------------------" << endl;
  //cout << "K: " << K_vec.transpose() << endl;
  //cout << "C: " << C.transpose() << endl;
  //cout << "xdes: " << x_desired.transpose() << endl;

  //vector<int> input_info_sizes{num_states, num_efforts, 3, 1};
  ////vector<int> input_params_sizes{num_states*num_efforts, num_efforts, num_states, 1};

  //auto info_connector = builder.AddSystem<InfoConnector>(num_positions, num_velocities, num_efforts);
  //auto multiplexer_info = builder.AddSystem<Multiplexer<double>>(input_info_sizes);

  //auto constant_zero_source_efforts = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(num_efforts));
  //auto constant_zero_source_imu = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(3));
  //auto constant_zero_source_timestamp = builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(1));

  //VectorXd params_vec(num_states*num_efforts + num_efforts + num_states);
  //params_vec << K_vec, C, x_desired;
  //AffineParams params(num_states, num_efforts);
  //params.SetDataVector(params_vec);

  //auto constant_params_source = builder.AddSystem<ConstantVectorSource<double>>(params);
  //auto control_output = builder.AddSystem<SubvectorPassThrough<double>>(
  //        (clqr_controller->get_output_port(0)).size(), 0, (clqr_controller->get_output_port(0)).size() - 1);
  //auto float_to_fixed_passthrough = builder.AddSystem<FloatToFixedConnector>(
  //    num_positions, num_velocities, num_efforts, 6);

  //builder.Connect(plant->state_output_port(), multiplexer_info->get_input_port(0));
  //builder.Connect(constant_zero_source_efforts->get_output_port(), multiplexer_info->get_input_port(1));
  //builder.Connect(constant_zero_source_imu->get_output_port(), multiplexer_info->get_input_port(2));
  //builder.Connect(constant_zero_source_timestamp->get_output_port(), multiplexer_info->get_input_port(3));
  //builder.Connect(multiplexer_info->get_output_port(0), float_to_fixed_passthrough->get_input_port(0));
  //builder.Connect(float_to_fixed_passthrough->get_output_port(0), info_connector->get_input_port(0));
  //builder.Connect(info_connector->get_output_port(0), clqr_controller->get_input_port_info());
  //builder.Connect(constant_params_source->get_output_port(), clqr_controller->get_input_port_params());
  //builder.Connect(clqr_controller->get_output_port(0), control_output->get_input_port());
  //builder.Connect(control_output->get_output_port(), plant->actuator_command_input_port()); 

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context = diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
  
  drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
  state.SetFromVector(x_sol);
  
  auto zero_input = Eigen::MatrixXd::Zero(num_efforts,1);
  context.FixInputPort(0, zero_input);
  
  //simulator.set_publish_every_time_step(false);
  //simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  
  lcm.StartReceiveThread();
  
  //simulator.StepTo(std::numeric_limits<double>::infinity());
  simulator.StepTo(0.000000001);
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
