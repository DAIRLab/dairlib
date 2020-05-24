//
// Created by jianshu on 5/20/20.
//

#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <bits/stdc++.h>  // system call
#include <cmath>
#include <numeric> // std::accumulate
#include <tuple>
#include <Eigen/QR>  // CompleteOrthogonalDecomposition

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"
#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/initial_guess.h"
#include "systems/goldilocks_models/file_utils.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::AutoDiffXd;
using dairlib::FindResourceOrThrow;


namespace dairlib::goldilocks_models {
// Robot models
DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
// Reduced order models
DEFINE_int32(rom_option, -1, "");

// inner loop
DEFINE_string(init_file, "", "Initial Guess for Trajectory Optimization");
DEFINE_double(major_feasibility_tol, 1e-4,
"nonlinear constraint violation tol");
DEFINE_int32(
    max_inner_iter, 150,
"Max iteration # for traj opt. Sometimes, snopt takes very small steps "
"(TODO: find out why), so maybe it's better to stop at some iterations and "
"resolve again.");
DEFINE_int32(n_node, -1, "# of nodes for traj opt");
DEFINE_double(eps_regularization, 1e-8, "Weight of regularization term"); //1e-4

//tasks
DEFINE_bool(is_zero_touchdown_impact, false,
"No impact force at fist touchdown");
DEFINE_bool(is_add_tau_in_cost, true, "Add RoM input in the cost function");

//others
DEFINE_string(
    program_name, "",
"The name of the program (to keep a record for future references)");

void createMBP(MultibodyPlant<double>* plant, int robot_option) {
  if (robot_option == 0) {
    Parser parser(plant);
    std::string full_name = FindResourceOrThrow(
        "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(
        -9.81 * Eigen::Vector3d::UnitZ());
    plant->WeldFrames(
        plant->world_frame(), plant->GetFrameByName("base"),
        drake::math::RigidTransform<double>());
    plant->Finalize();

  } else if (robot_option == 1) {
    Parser parser(plant);
    string full_name =
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(
        -9.81 * Eigen::Vector3d::UnitZ());
    plant->Finalize();
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setCostWeight(double* Q, double* R, double* all_cost_scale,
                   int robot_option) {
  if (robot_option == 0) {
    *Q = 1;
    *R = 0.1;
    //*all_cost_scale = 1;  // not implemented yet
  } else if (robot_option == 1) {
    *Q = 5 * 0.1;
    *R = 0.1 * 0.01;
    *all_cost_scale = 0.2/* * 0.12*/;
  }
}
void setRomDim(int* n_s, int* n_tau, int rom_option) {
  if (rom_option == 0) {
    // 2D -- lipm
    *n_s = 2;
    *n_tau = 0;
  } else if (rom_option == 1) {
    // 4D -- lipm + swing foot
    *n_s = 4;
    *n_tau = 2;
  } else if (rom_option == 2) {
    // 1D -- fix com vertical acceleration
    *n_s = 1;
    *n_tau = 0;
  } else if (rom_option == 3) {
    // 3D -- fix com vertical acceleration + swing foot
    *n_s = 3;
    *n_tau = 2;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setRomBMatrix(MatrixXd* B_tau, int rom_option) {
  if ((rom_option == 0) || (rom_option == 2)) {
    // passive rom, so we don't need B_tau
  }
  else if (rom_option == 1) {
    DRAKE_DEMAND(B_tau->rows() == 4);
    (*B_tau)(2, 0) = 1;
    (*B_tau)(3, 1) = 1;
  }
  else if (rom_option == 3) {
    DRAKE_DEMAND(B_tau->rows() == 3);
    (*B_tau)(1, 0) = 1;
    (*B_tau)(2, 1) = 1;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setInitialTheta(VectorXd& theta_s, VectorXd& theta_sDDot,
                     int n_feature_s, int rom_option) {
  // // Testing intial theta
  // theta_s = 0.25*VectorXd::Ones(n_theta_s);
  // theta_sDDot = 0.5*VectorXd::Ones(n_theta_sDDot);
  // theta_s = VectorXd::Random(n_theta_s);
  // theta_sDDot = VectorXd::Random(n_theta_sDDot);

  if (rom_option == 0) {
    // 2D -- lipm
    theta_s(0) = 1;
    theta_s(1 + n_feature_s) = 1;
    theta_sDDot(0) = 1;
  } else if (rom_option == 1) {
    // 4D -- lipm + swing foot
    theta_s(0) = 1;
    theta_s(1 + n_feature_s) = 1;
    theta_s(2 + 2 * n_feature_s) = 1;
    theta_s(3 + 3 * n_feature_s) = 1;
    theta_sDDot(0) = 1;
  } else if (rom_option == 2) {
    // 1D -- fix com vertical acceleration
    theta_s(1) = 1;
  } else if (rom_option == 3) {
    // 3D -- fix com vertical acceleration + swing foot
    theta_s(1) = 1;
    theta_s(2 + 1 * n_feature_s) = 1;
    theta_s(3 + 2 * n_feature_s) = 1;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}

// trajectory optimization for given task and model
void trajOptGivenModel(double stride_length, double ground_incline,
    double turning_rate,const string dir,int num,int sample_idx){
  // Create MBP
  MultibodyPlant<double> plant(0.0);
  createMBP(&plant, FLAGS_robot_option);

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);
  cout << endl;

  // Parameters for the inner loop optimization
  int max_inner_iter = FLAGS_max_inner_iter;
  if (FLAGS_robot_option == 0) {
    max_inner_iter = 300;
  }
  double Q = 0; // Cost on velocity
  double R = 0;  // Cost on input effort
  double all_cost_scale = 1;
  setCostWeight(&Q, &R, &all_cost_scale, FLAGS_robot_option);
  int n_node = 20;
  if (FLAGS_robot_option == 0) {
    n_node = 20;
  } else if (FLAGS_robot_option == 1) {
    n_node = 20;
  }
  if (FLAGS_n_node > 0) n_node = FLAGS_n_node;
  if (FLAGS_robot_option == 1) {
    // If the node density is too low, it's harder for SNOPT to converge well.
    // The ratio of distance per nodes = 0.2/16 is fine for snopt, but
    // 0.3 / 16 is too high.
    // However, currently it takes too much time to compute with many nodes, so
    // we try 0.3/24.
    double max_distance_per_node = 0.3 / 16;
//    DRAKE_DEMAND((max_stride_length / n_node) <= max_distance_per_node);
  }

  // Reduced order model parameters
  int rom_option = (FLAGS_rom_option >= 0) ? FLAGS_rom_option : 0;
  int n_s = 0;
  int n_tau = 0;
  setRomDim(&n_s, &n_tau, rom_option);
  int n_sDDot = n_s; // Assume that are the same (no quaternion)
  MatrixXd B_tau = MatrixXd::Zero(n_sDDot, n_tau);
  setRomBMatrix(&B_tau, rom_option);

  // Reduced order model setup
  KinematicsExpression<double> kin_expression(n_s, 0, &plant, FLAGS_robot_option);
  DynamicsExpression dyn_expression(n_sDDot, 0, rom_option, FLAGS_robot_option);
  VectorXd dummy_q = VectorXd::Ones(plant.num_positions());
  VectorXd dummy_s = VectorXd::Ones(n_s);
  int n_feature_s = kin_expression.getFeature(dummy_q).size();
  int n_feature_sDDot =
      dyn_expression.getFeature(dummy_s, dummy_s).size();
  int n_theta_s = n_s * n_feature_s;
  int n_theta_sDDot = n_sDDot * n_feature_sDDot;
  VectorXd theta_s(n_theta_s);
  VectorXd theta_sDDot(n_theta_sDDot);

  // Initial guess of theta
  theta_s = VectorXd::Zero(n_theta_s);
  theta_sDDot = VectorXd::Zero(n_theta_sDDot);
  setInitialTheta(theta_s, theta_sDDot, n_feature_s, rom_option);

  double duration = 0.4;
  if (FLAGS_robot_option == 0) {
    duration = 0.746;  // Fix the duration now since we add cost ourselves
  } else if (FLAGS_robot_option == 1) {
    duration = 0.4; // 0.4;
  }

  bool is_get_nominal = false;
  int max_inner_iter_pass_in = is_get_nominal ? 200 : max_inner_iter;

  string init_file_pass_in = "";
  string prefix = to_string(num) +  "_" + to_string(sample_idx) + "_";

  // Vectors/Matrices for the outer loop
  int N_sample = 1;
  vector<std::shared_ptr<VectorXd>> w_sol_vec(N_sample);
  vector<std::shared_ptr<MatrixXd>> H_vec(N_sample);
  vector<std::shared_ptr<VectorXd>> b_vec(N_sample);
  vector<std::shared_ptr<VectorXd>> c_vec(N_sample);
  vector<std::shared_ptr<MatrixXd>> A_vec(N_sample);
  vector<std::shared_ptr<VectorXd>> lb_vec(N_sample);
  vector<std::shared_ptr<VectorXd>> ub_vec(N_sample);
  vector<std::shared_ptr<VectorXd>> y_vec(N_sample);
  vector<std::shared_ptr<MatrixXd>> B_vec(N_sample);
  vector<std::shared_ptr<int>> is_success_vec(N_sample);
  for (int i = 0; i < N_sample; i++) {
    w_sol_vec[i] = std::make_shared<VectorXd>();
    H_vec[i] = std::make_shared<MatrixXd>();
    b_vec[i] = std::make_shared<VectorXd>();
    c_vec[i] = std::make_shared<VectorXd>();
    A_vec[i] = std::make_shared<MatrixXd>();
    lb_vec[i] = std::make_shared<VectorXd>();
    ub_vec[i] = std::make_shared<VectorXd>();
    y_vec[i] = std::make_shared<VectorXd>();
    B_vec[i] = std::make_shared<MatrixXd>();
    is_success_vec[i] = std::make_shared<int>();
  }

  vector<std::shared_ptr<int>> thread_finished_vec(N_sample);
  for (int i = 0; i < N_sample; i++) {
    thread_finished_vec[i] = std::make_shared<int>(0);
  }

  bool extend_model_this_iter = false;
  int n_rerun = 1;
  double cost_threshold_for_update = std::numeric_limits<double>::infinity();
  int N_rerun = 0;

  //run trajectory optimization
  trajOptGivenWeights(std::ref(plant), std::ref(plant_autoDiff),
                      n_s, n_sDDot, n_tau,
                      n_feature_s, n_feature_sDDot, std::ref(B_tau),
                      std::ref(theta_s), std::ref(theta_sDDot),
                      stride_length, ground_incline, turning_rate,
                      duration, n_node, max_inner_iter_pass_in,
                      FLAGS_major_feasibility_tol, FLAGS_major_feasibility_tol,
                      std::ref(dir), init_file_pass_in, prefix,
                      std::ref(w_sol_vec),
                      std::ref(A_vec),
                      std::ref(H_vec),
                      std::ref(y_vec),
                      std::ref(lb_vec),
                      std::ref(ub_vec),
                      std::ref(b_vec),
                      std::ref(c_vec),
                      std::ref(B_vec),
                      std::ref(is_success_vec),
                      std::ref(thread_finished_vec),
                      Q, R, all_cost_scale,
                      FLAGS_eps_regularization,
                      is_get_nominal,
                      FLAGS_is_zero_touchdown_impact,
                      extend_model_this_iter,
                      FLAGS_is_add_tau_in_cost,
                      sample_idx, n_rerun,
                      cost_threshold_for_update, N_rerun,
                      rom_option,
                      FLAGS_robot_option);
}

//naive test function for search algorithm
int sample_result(double stride_length,double ground_incline,
    double turning_rate){
  int result = 0;
  if( (ground_incline<0.12) && (ground_incline>-0.12)){
    if((stride_length<0.25) && (stride_length>0.15)){
      result = 1;
    }
  }
  return result;
}


//search the max/min ground incline for fixed stride length
double boundary_for_one_dimension(int max_iteration,double stride_length,
    double gi_low,double gi_high,double turning_rate,double resolution,
    const string dir,int num){
  int sample_idx = 0;
  cout << "sample# (rerun #) | stride | incline | turning | init_file | "
          "Status | Solve time | Cost (tau cost)\n";
  for (sample_idx = 0; sample_idx <= max_iteration; sample_idx++){
    //run trajectory optimization and judge the solution
    trajOptGivenModel(stride_length, gi_high,
        turning_rate, dir, num, sample_idx);
    string prefix = to_string(num) +  "_" + to_string(sample_idx) + "_";
    //store stride length, ground incline and turning rate
    writeCSV(dir + prefix +
        string("stride_length.csv"), stride_length*MatrixXd::Ones(1,1));
    writeCSV(dir + prefix +
        string("ground_incline.csv"), gi_high*MatrixXd::Ones(1,1));
    writeCSV(dir + prefix +
        string("turning_rate.csv"), turning_rate*MatrixXd::Ones(1,1));

    int sample_success =
        (readCSV(dir + prefix + string("is_success.csv")))(0, 0);

//    //test search algorithm
//    int sample_success = sample_result(stride_length,high,turning_rate);

    if (sample_success){
      gi_low = gi_high;
      gi_high = 2*gi_high;
    }
    else{
      gi_high = (gi_high+gi_low)/2;
    }
    if(abs(gi_high-gi_low) < resolution){
      break;
    }
  }
  return gi_low;
}

//extend range of stride length
int extend_range(const string dir,
    double stride_length_0,int max_iter,double ground_incline_0,
    double ground_incline_high,double ground_incline_low,
    double ground_incline_resolution,double turning_rate_0,
    int boundary_num,int update_direction,
    double delta_stride_length){
  //keep updating stride_length and search boundary of ground incline for each
  //stride length
  int iter;
  double stride_length = stride_length_0;
  for (iter = 0; iter <= max_iter; iter++){
    //fix stride length and find max ground incline
    double max_gi = boundary_for_one_dimension(max_iter, stride_length,
        ground_incline_0, ground_incline_high,
        turning_rate_0, ground_incline_resolution,
        dir, boundary_num);
    VectorXd boundary_point(2);
    boundary_point<<stride_length,max_gi;
    writeCSV(dir + to_string(boundary_num) +  "_" +
        string("max_ground_incline.csv"), boundary_point);
    cout << "boundary point index | stride length | ground incline"<<endl;
    cout<<" \t "<<boundary_num<< "\t" <<" | "<<"\t"<<stride_length
        <<"\t"<<" | "<<"\t"<<max_gi<<endl;
    boundary_num += 1;

    //find min ground incline
    double min_gi = boundary_for_one_dimension(max_iter, stride_length,
        ground_incline_0, ground_incline_low,
        turning_rate_0, ground_incline_resolution,
        dir, boundary_num);
    boundary_point<<stride_length,min_gi;
    writeCSV(dir + to_string(boundary_num) +  "_" +
        string("min_ground_incline.csv"), boundary_point);
    cout << "boundary point index | stride length | ground incline"<<endl;
    cout<<" \t "<<boundary_num<< "\t" <<" | "<<"\t"<<stride_length
        <<"\t"<<" | "<<"\t"<<min_gi<<endl;
    boundary_num += 1;
    //if max_gi and min_gi are same, find the boundary of stride length
    if(min_gi==max_gi){
      break;
    }
    //else continue search
    else{
      if(update_direction==1){
        stride_length = stride_length+delta_stride_length;
      }
      if(update_direction==0){
        stride_length = stride_length-delta_stride_length;
      }
    }
  }
  return boundary_num;
}

int find_boundary(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "Trail name: " << FLAGS_program_name << endl;
  cout << "Git commit hash: " << endl;
  std::system("git rev-parse HEAD");
  cout << "Result of \"git diff-index HEAD\":" << endl;
  std::system("git diff-index HEAD");
  cout << endl;

  const string dir = "../dairlib_data/goldilocks_models/find_boundary/robot_" +
      to_string(FLAGS_robot_option) + "/";
  if (!CreateFolderIfNotExist(dir)) return 0;

  /*
   * initialize task space
   */
  cout << "\nInitialize task space:\n";
  double stride_length_0 = 0.2;
  double delta_stride_length = 0.01;
  cout<<"initial stride length "<<stride_length_0<<endl;
  cout<<"delta stride length "<<delta_stride_length<<endl;

  double ground_incline_0 = 0;
  double initial_ground_incline_max = 0.1;
  double initial_ground_incline_min = -0.1;
  double ground_incline_resolution = 0.01;
  cout<<"max ground incline initially located within "<<ground_incline_0
      <<" to "<<initial_ground_incline_max<<endl;
  cout<<"min ground incline initially located within "<<initial_ground_incline_min
      <<" to "<<ground_incline_0<<endl;
  cout<<"ground incline resolution "<<ground_incline_resolution<<endl;

  double turning_rate_0 = 0;
//  double delta_turning_rate = 0;

  /*
   * start iteration
   */
  cout << "\nStart iteration:\n";
  int max_iter = 50;
  int boundary_sample_num = 0;
  int extend_direction;//0:decrease the stride length;1:increase stride length

  //search max stride length
  cout<<"search along increasing the stride length"<<endl;
  extend_direction = 1;
  boundary_sample_num = extend_range(dir,
      stride_length_0,max_iter, ground_incline_0,
      initial_ground_incline_max,initial_ground_incline_min,
      ground_incline_resolution,turning_rate_0,
      boundary_sample_num, extend_direction,
      delta_stride_length);

  //search min stride length
  cout<<"search along decreasing the stride length"<<endl;
  extend_direction = 0;
  boundary_sample_num = extend_range(dir,
      stride_length_0,max_iter, ground_incline_0,
      initial_ground_incline_max,initial_ground_incline_min,
      ground_incline_resolution,turning_rate_0,
      boundary_sample_num, extend_direction,
      delta_stride_length);

  return 0;
}
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::find_boundary(argc, argv);
}