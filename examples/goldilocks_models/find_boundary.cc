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
DEFINE_bool(is_get_nominal,false,"is calculating the cost without ROM constraints");
DEFINE_bool(use_optimized_model,false,"read theta from files to apply optimized model");

//tasks
DEFINE_bool(is_zero_touchdown_impact, false,
"No impact force at fist touchdown");
DEFINE_bool(is_add_tau_in_cost, true, "Add RoM input in the cost function");

//outer loop
DEFINE_int32(max_outer_iter, 150 , "max number of iterations for searching on each "
                                "direction of one dimension");

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

//read theta from files to use optimized model
void readThetaFromFiles(const string dir,int theta_idx,
    VectorXd& theta_s, VectorXd& theta_sDDot){
  theta_s = readCSV(dir + to_string(theta_idx) + string("_theta_s.csv"));
  theta_sDDot = readCSV(dir + to_string(theta_idx) + string("_theta_sDDot.csv"));
}

//use interpolation to set the initial guess for the trajectory optimization
string getInitFileName(const string directory, int traj_opt_num,
    bool is_rerun,int rerun_traj_idx=-1){
  int gamma_dimension = 3;
  VectorXd current_gamma = readCSV(directory + to_string(traj_opt_num)
                                       + string("_0_gamma.csv"));
  string initial_file_name;
  if(is_rerun){
    //if not specify which Traj Opt result to use, use its own result to rerun;
    //else use the specified one.
    if(rerun_traj_idx==-1) {
      initial_file_name = to_string(traj_opt_num)
          + string("_0_w.csv");
    }
    else{
      initial_file_name = to_string(rerun_traj_idx)
          + string("_0_w.csv");
    }
  }else{
    if(traj_opt_num==0){
      initial_file_name = "";
    }else{
      VectorXd initial_guess;
      //take out corresponding w and calculate the weight for interpolation
      MatrixXd w_gamma;
      VectorXd weight_gamma;
      //paras used to decide gamma scale
      double delta_sl = 0.015;
      double delta_gi = 0.05;
      double delta_tr = 0.125;
      //calculate the weighted sum of past solutions
      int sample_num = 0;
      for (sample_num = 0; sample_num < traj_opt_num; sample_num++) {
        //check if this sample is success
        int is_success = (readCSV(directory + to_string(sample_num)
                                      + string("_0_is_success.csv")))(0,0);
        if(is_success == 1) {
          //extract past gamma
          VectorXd past_gamma = readCSV(directory + to_string(sample_num)
                                            + string("_0_gamma.csv"));
          //calculate the weight for each sample using the third power of the difference between gamma
          VectorXd gamma_scale(gamma_dimension);
          gamma_scale << 1/delta_gi,1/delta_sl,1.3/delta_tr;
          VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*gamma_scale.array();
          VectorXd dif_gamma2 = dif_gamma.array().pow(2);
          double distance_gamma =  (dif_gamma.transpose() * dif_gamma2)(0,0);
          //extract the solution of this sample
          VectorXd w_to_interpolate = readCSV(directory + to_string(sample_num)
                                                  + string("_0_w.csv"));
          //concatenate the weight and solution for further calculation
          w_gamma.conservativeResize(w_to_interpolate.rows(),w_gamma.cols()+1);
          w_gamma.col(w_gamma.cols()-1) = w_to_interpolate;
          weight_gamma.conservativeResize(weight_gamma.rows()+1);
          weight_gamma(weight_gamma.rows()-1)= 1 /distance_gamma;
        }
      }
      DRAKE_DEMAND(weight_gamma.rows()>0);
      //    normalize weight
      weight_gamma = weight_gamma / weight_gamma.sum();
      initial_guess = w_gamma * weight_gamma;
      //    save initial guess and set init file
      initial_file_name = to_string(traj_opt_num)
          + string("_0_initial_guess.csv");
      writeCSV(directory + initial_file_name, initial_guess);
    }
  }

  return initial_file_name;
}

// trajectory optimization for given task and model
void trajOptGivenModel(double stride_length, double ground_incline,
    double turning_rate,const string dir,int num,bool is_rerun,
    int initial_guess_idx=-1){
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
  if(FLAGS_use_optimized_model){
    //you have to specify the theta to use
    int theta_idx = 100;
    const string dir_find_models = "../dairlib_data/goldilocks_models/find_models/robot_" +
        to_string(FLAGS_robot_option) + "/";
    readThetaFromFiles(dir_find_models, theta_idx, theta_s, theta_sDDot);
  }
  else{
    setInitialTheta(theta_s, theta_sDDot, n_feature_s, rom_option);
  }

  double duration = 0.4;
  if (FLAGS_robot_option == 0) {
    duration = 0.746;  // Fix the duration now since we add cost ourselves
  } else if (FLAGS_robot_option == 1) {
    duration = 0.4; // 0.4;
  }

  bool is_get_nominal = FLAGS_is_get_nominal;
  int max_inner_iter_pass_in = is_get_nominal ? 200 : max_inner_iter;

//  string init_file_pass_in = "";
  string init_file_pass_in = getInitFileName(dir, num, is_rerun,
      initial_guess_idx);
  int sample_idx = 0;
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
  int n_rerun = 0;
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
double sample_result(double stride_length,double ground_incline,
    double turning_rate){
  double result = std::numeric_limits<double>::infinity();
  if( (ground_incline<0.12) && (ground_incline>-0.12)){
    if((stride_length<0.35) && (stride_length>0.25)){
      result = 0;
    }
  }
  return result;
}


//search the boundary point along one direction
void boundary_for_one_direction(const string dir,int dims,int max_iteration,
    VectorXd init_gamma,VectorXd step_direction, VectorXd step_size,
    double max_cost,int& traj_num,int& boundary_point_idx){
  int iter;
  int sample_idx = 0;
  VectorXd new_gamma(dims);
  VectorXd last_gamma = init_gamma;
  VectorXd boundary_point(dims);
  VectorXd step = step_size.array()*step_direction.array();
  MatrixXd cost_list;
  double decay_factor;//take a large step at the beginning
  cout << "sample# (rerun #) | stride | incline | turning | init_file | "
          "Status | Solve time | Cost (tau cost)\n";
  for (iter = 1; iter <= max_iteration; iter++){
    decay_factor = 2.5*pow(0.95,iter);
    if(decay_factor>1){
      new_gamma = last_gamma+decay_factor*step;
      last_gamma = new_gamma;
    }
    else{
      new_gamma = last_gamma+step;
      last_gamma = new_gamma;
    }
    //if stride length is negative or zero,stop searching
    if(new_gamma[0]<=0){
      boundary_point_idx += 1;
      boundary_point = new_gamma-step;
      writeCSV(dir + to_string(boundary_point_idx) +  "_" +
          string("boundary_point.csv"), boundary_point);
      cout << "boundary point index | stride length | ground incline"
              " | turning rate"<<endl;
      cout<<" \t "<<boundary_point_idx<< "\t" <<" | "<<"\t"<<boundary_point[0]
          <<"\t"<<" | "<<"\t"<<boundary_point[1]<<"\t"<<" | "<<"\t"<<
          boundary_point[2]<<endl;
      break;
    }
    //store stride length, ground incline and turning rate
    traj_num += 1;
    string prefix = to_string(traj_num) +  "_" + to_string(sample_idx) + "_";
    writeCSV(dir + prefix +
        string("gamma.csv"), new_gamma);
    //run trajectory optimization and judge the solution
    trajOptGivenModel(new_gamma[0], new_gamma[1],
        new_gamma[2], dir, traj_num, false);
    //check if snopt find a solution successfully. If not, rerun the Traj Opt
    int is_success = (readCSV(dir + prefix + string("is_success.csv")))(0, 0);
    if(is_success==0){
      trajOptGivenModel(new_gamma[0], new_gamma[1],
                        new_gamma[2], dir, traj_num,true);
    }
    double sample_cost =
        (readCSV(dir + prefix + string("c.csv")))(0, 0);
    // without a good initial guess, the initial point is easily stuck in a local minimum
    // use the first sample to judge the solution of initial point
    if(iter==1){
      double initial_cost =
          (readCSV(dir + string("0_0_c.csv")))(0, 0);
      if(initial_cost>1.2*sample_cost){
        trajOptGivenModel(init_gamma[0], init_gamma[1],
                          init_gamma[2], dir, 0,true,traj_num);
      }
    }
    //save the trajectory optimization index and corresponding cost for further use
    cost_list.conservativeResize(cost_list.rows()+1, 2);
    cost_list.row(cost_list.rows()-1)<<traj_num,sample_cost;

    //test search algorithm
//    double sample_cost = sample_result(new_gamma[0],new_gamma[1],
//        new_gamma[2]);

    if(sample_cost>max_cost){
      boundary_point_idx += 1;
      boundary_point = new_gamma-step;
      writeCSV(dir + to_string(boundary_point_idx) +  "_" +
          string("boundary_point.csv"), boundary_point);
      cout << "boundary point index | stride length | ground incline"
              " | turning rate"<<endl;
      cout<<" \t "<<boundary_point_idx<< "\t" <<" | "<<"\t"<<boundary_point[0]
          <<"\t"<<" | "<<"\t"<<boundary_point[1]<<"\t"<<" | "<<"\t"<<
          boundary_point[2]<<endl;
      break;
    }
  }
  cout << "\nStart checking the cost:\n";
  //check the adjacent sample to avoid being stuck in local minimum
  int traj_idx;
  for(iter=cost_list.rows()-2;iter>=1;iter--){
    traj_idx = cost_list(iter,0);
    //if cost is larger than adjacent sample, rerun with adjacent sample result
    if( (cost_list(iter,1) > 1.2*cost_list(iter-1,1)) &&
      (cost_list(iter,1) > 1.2*cost_list(iter+1,1)) ){
      VectorXd gamma_to_rerun = readCSV(dir + to_string(traj_idx)
                                           + string("_0_gamma.csv"));
      //choose the result of sample with lower cost as initial guess
      if(cost_list(iter-1,1)<cost_list(iter+1,1)){
        trajOptGivenModel(gamma_to_rerun[0], gamma_to_rerun[1],
                          gamma_to_rerun[2], dir, traj_idx,
                          true, traj_idx-1);
      }
      else{
        trajOptGivenModel(gamma_to_rerun[0], gamma_to_rerun[1],
                          gamma_to_rerun[2], dir, traj_idx,
                          true, traj_idx+1);
      }
      //update cost list
      cost_list(iter,1) = readCSV(dir + to_string(iter)
                                      + string("_0_c.csv"))(0,0);
    }
  }
  writeCSV(dir +  to_string(boundary_point_idx)  +
      string("_cost_list.csv"), cost_list);
  cout << "\nFinish checking the cost:\n";
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
  int dimensions = 3;//dimension of the task space
  double stride_length_0 = 0;
  if(FLAGS_robot_option==0){
    stride_length_0 = 0.2;
  }
  else if(FLAGS_robot_option==1){
      stride_length_0 = 0.2;
  }
  double delta_stride_length = 0.01;
  cout<<"initial stride length "<<stride_length_0<<endl;
  cout<<"delta stride length "<<delta_stride_length<<endl;

  double ground_incline_0 = 0;
  double delta_ground_incline = 0.01;
  cout<<"initial ground incline "<<ground_incline_0<<endl;
  cout<<"delta ground incline "<<delta_ground_incline<<endl;

  double turning_rate_0 = 0;
  double delta_turning_rate = 0;
  cout<<"initial turning rate "<<turning_rate_0<<endl;
  cout<<"delta turning rate "<<delta_turning_rate<<endl;

  VectorXd initial_gamma(dimensions);
  initial_gamma<<stride_length_0,ground_incline_0,turning_rate_0;
  VectorXd delta(dimensions);
  delta<<delta_stride_length,delta_ground_incline,delta_turning_rate;


  /*
   * Iteration setting
   */
  cout << "\nIteration setting:\n";
  cout<<"get nominal cost: "<<FLAGS_is_get_nominal<<endl;
  cout<<"use optimized model: "<<FLAGS_use_optimized_model<<endl;
  int max_iter = FLAGS_max_outer_iter;
  //TODO:decide the threshold under different situation
  double cost_threshold = 20;
  if(FLAGS_robot_option==0)
  {
    if(FLAGS_is_get_nominal){
      cost_threshold = 35;
    }
    else{
      cost_threshold = 30;
    }
  }
  else{
    if(FLAGS_is_get_nominal){
      cost_threshold = 20;
    }
    else{
      cost_threshold = 15;
    }
  }
  cout<<"cost_threshold "<<cost_threshold<<endl;
  int boundary_sample_num = 0;//use this to set the index for boundary point
  int traj_opt_num = 0;//use this to set the index for Traj Opt
  VectorXd extend_direction(dimensions);//the direction of searching

  //create components for each dimension used to decide the direction
  int sl_comp_num = 5;
  VectorXd e_sl(sl_comp_num);
  e_sl<<0,-1,-0.5,0.5,1;

  int gi_comp_num = 5;
  VectorXd e_gi(gi_comp_num);
  e_gi<<0,-1,-0.5,0.5,1;

  int tr_comp_num = 1;
  VectorXd e_tr(tr_comp_num);
  e_tr<<0;

  /*
   * start iteration
   */
  //evaluate initial point
  cout << "\nCalculate Central Point Cost:\n";
  writeCSV(dir +  to_string(traj_opt_num)  +
      string("_0_gamma.csv"), initial_gamma);
  cout << "sample# (rerun #) | stride | incline | turning | init_file | "
          "Status | Solve time | Cost (tau cost)\n";
  trajOptGivenModel(stride_length_0, ground_incline_0,
                    turning_rate_0, dir, traj_opt_num, false);
  //rerun the Traj Opt to avoid being stuck in local minimum
  trajOptGivenModel(stride_length_0, ground_incline_0,
                    turning_rate_0, dir, traj_opt_num, true);

  int dim1,dim2,dim3;
  VectorXd step(dimensions);
  //for all the direction, search the boundary
  for (dim1=0;dim1<sl_comp_num;dim1++){
    for(dim2=0;dim2<gi_comp_num;dim2++){
      for(dim3=0;dim3<tr_comp_num;dim3++){
        extend_direction << e_sl[dim1],e_gi[dim2],e_tr[dim3];
        //filter with infinity norm to avoid repetitive searching
        //search along the direction
        if( (extend_direction.lpNorm<Eigen::Infinity>()) >=1){
          cout << "Start searching along direction: ["<<extend_direction[0]
               <<","<<extend_direction[1]<<","<<extend_direction[2]<<"]"<<endl;
          //normalize the direction vector
          //extend_direction = extend_direction/extend_direction.norm();
          boundary_for_one_direction(dir,dimensions,max_iter,
              initial_gamma,extend_direction,delta,
              cost_threshold,traj_opt_num,boundary_sample_num);
        }
      }
    }
  }

  return 0;
}
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::find_boundary(argc, argv);
}