#include <gflags/gflags.h>
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <bits/stdc++.h>  // system call
#include <cmath>
#include <tuple>
#include <Eigen/QR>  // CompleteOrthogonalDecomposition

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/find_models/initial_guess.h"
#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/task.h"
#include "systems/goldilocks_models/file_utils.h"
#include "examples/goldilocks_models/find_boundary_utils/search_setting.h"

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
DEFINE_double(major_optimality_tol, 1e-4,
"tolerance for optimality condition (complementarity gap)");
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
DEFINE_int32(theta_index,-1,"# of optimized model to use");

//tasks
DEFINE_bool(is_zero_touchdown_impact, false,
"No impact force at fist touchdown");
DEFINE_bool(is_add_tau_in_cost, true, "Add RoM input in the cost function");

//outer loop
DEFINE_int32(max_outer_iter, 150 , "max number of iterations for searching on each "
                                "direction of one dimension");
DEFINE_bool(search_sl,true,"decide whether to search the stride length");
DEFINE_bool(search_gi,true,"decide whether to search the ground incline");
DEFINE_bool(search_v,false,"decide whether to search the velocity");
DEFINE_bool(search_tr,false,"decide whether to search the turning rate");

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
void ReadThetaFromFiles(const string dir,int theta_idx,
    VectorXd& theta_y, VectorXd& theta_yddot){
  theta_y = readCSV(dir + to_string(theta_idx) + string("_theta_y.csv"));
  theta_yddot = readCSV(dir + to_string(theta_idx) + string("_theta_yddot.csv"));
}


//use interpolation to set the initial guess for the trajectory optimization
string getInitFileName(const string directory, int traj_opt_num,
    bool is_rerun,int rerun_traj_idx=-1){
  VectorXd current_gamma = readCSV(directory + to_string(traj_opt_num)
                                       + string("_0_task.csv"));
  int gamma_dimension = current_gamma.size();
  VectorXd gamma_scale(gamma_dimension);
  //paras used to decide gamma scale
  if(FLAGS_robot_option==0)
  {
    double delta_sl = 0.015;
    double delta_gi = 0.05;
    double delta_v = 0.02;
    gamma_scale << 1/delta_sl,1/delta_gi,1/delta_v;

  }
  else if(FLAGS_robot_option==1)
  {
    double delta_sl = 0.015;
    double delta_gi = 0.05;
    double delta_tr = 0.125;
    double delta_v = 0.02;
    gamma_scale << 1/delta_sl,1/delta_gi,1/delta_v,1.3/delta_tr;
  }
  string initial_file_name;
  if(is_rerun){
    //if not specify which Traj Opt solution to use, use its own result to rerun;
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
    VectorXd initial_guess;
    //take out corresponding w and calculate the weight for interpolation
    MatrixXd w_gamma;
    VectorXd weight_gamma;
    int sample_num = 0;
    string prefix;
    // initial guess for initial point
    if(traj_opt_num==0){
      //specify initial guess if using optimized model
      //use solutions during ROM optimization process to calculate the initial guess
      if(FLAGS_use_optimized_model){
        const string dir_find_models = "../dairlib_data/goldilocks_models/find_models/robot_" +
            to_string(FLAGS_robot_option) + "/";
        //calculate the weighted sum of solutions
        int theta_idx = FLAGS_theta_index;
        while(file_exist(dir_find_models + to_string(theta_idx)+ '_' +
        to_string(sample_num)+ string("_is_success.csv"))){
          prefix = to_string(theta_idx)+"_"+to_string(sample_num);
          //calculate interpolation weight and extract solutions
          InterpolateAmongDifferentTasks(dir_find_models, prefix,
              current_gamma,gamma_scale,weight_gamma,w_gamma);
          sample_num = sample_num+1;
        }
        //calculate interpolated initial guess
        initial_guess = CalculateInterpolation(weight_gamma,
            w_gamma);
        //    save initial guess and set init file
        initial_file_name = to_string(traj_opt_num)
            + string("_0_initial_guess.csv");
        writeCSV(directory + initial_file_name, initial_guess);
      }
      else{
        initial_file_name = "";
      }
    }else{
      //use past solutions to calculate interpolated initial guess
      //calculate the weighted sum of past solutions
      for (sample_num = 0; sample_num < traj_opt_num; sample_num++) {
        prefix = to_string(sample_num)+"_"+to_string(0);
        InterpolateAmongDifferentTasks(directory, prefix,
            current_gamma,gamma_scale,weight_gamma,w_gamma);
      }
      initial_guess = CalculateInterpolation(weight_gamma,
                                             w_gamma);
      //    save initial guess and set init file
      initial_file_name = to_string(traj_opt_num)
          + string("_0_initial_guess.csv");
      writeCSV(directory + initial_file_name, initial_guess);
    }
  }

  return initial_file_name;
}

// trajectory optimization for given task and model
void TrajOptGivenModel(Task task, const string dir,int num,bool is_rerun,
    int initial_guess_idx=-1,bool turn_on_scaling=true){
  // Create MBP
  drake::logging::set_log_level("err");  // ignore warnings about joint limits
  MultibodyPlant<double> plant(0.0);
  createMBP(&plant, FLAGS_robot_option);

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);

  // Parameters for the inner loop optimization
  int max_inner_iter = FLAGS_max_inner_iter;
  if (FLAGS_robot_option == 0) {
    max_inner_iter = 300;
  }
  double Q = 0; // Cost on velocity
  double R = 0;  // Cost on input effort
  double all_cost_scale = 1;
  setCostWeight(&Q, &R, &all_cost_scale, FLAGS_robot_option);
  // Inner loop setup
  InnerLoopSetting inner_loop_setting = InnerLoopSetting();
  inner_loop_setting.Q_double = Q;
  inner_loop_setting.R_double = R;
  inner_loop_setting.eps_reg = FLAGS_eps_regularization;
  inner_loop_setting.all_cost_scale = all_cost_scale;
  inner_loop_setting.is_add_tau_in_cost = FLAGS_is_add_tau_in_cost;
  inner_loop_setting.is_zero_touchdown_impact = FLAGS_is_zero_touchdown_impact;
  inner_loop_setting.max_iter = max_inner_iter;
  inner_loop_setting.major_optimality_tol = FLAGS_major_optimality_tol;
  inner_loop_setting.major_feasibility_tol = FLAGS_major_feasibility_tol;
  inner_loop_setting.snopt_scaling = turn_on_scaling;
  inner_loop_setting.directory = dir;

  // Reduced order model parameters
  int n_y = 0;
  int n_tau = 0;
  setRomDim(&n_y, &n_tau, FLAGS_rom_option);
  int n_yddot = n_y; // Assume that are the same (no quaternion)
  MatrixXd B_tau = MatrixXd::Zero(n_yddot, n_tau);
  setRomBMatrix(&B_tau, FLAGS_rom_option);
  writeCSV(dir + string("B_tau.csv"), B_tau);

  // Reduced order model setup
  KinematicsExpression<double> kin_expression(n_y, 0, &plant, FLAGS_robot_option);
  DynamicsExpression dyn_expression(n_yddot, 0, FLAGS_rom_option, FLAGS_robot_option);
  VectorXd dummy_q = VectorXd::Ones(plant.num_positions());
  VectorXd dummy_s = VectorXd::Ones(n_y);
  int n_feature_y = kin_expression.getFeature(dummy_q).size();
  int n_feature_yddot =
      dyn_expression.getFeature(dummy_s, dummy_s).size();
  int n_theta_y = n_y * n_feature_y;
  int n_theta_yddot = n_yddot * n_feature_yddot;

  // Initial guess of theta
  VectorXd theta_y = VectorXd::Zero(n_theta_y);
  VectorXd theta_yddot = VectorXd::Zero(n_theta_yddot);
  if(FLAGS_use_optimized_model){
    //you have to specify the theta to use
    DRAKE_DEMAND(FLAGS_theta_index>=0);
    int theta_idx = FLAGS_theta_index;

    const string dir_find_models = "../dairlib_data/goldilocks_models/find_models/robot_" +
        to_string(FLAGS_robot_option) + "/";
    ReadThetaFromFiles(dir_find_models, theta_idx, theta_y, theta_yddot);
  }
  else{
    setInitialTheta(theta_y, theta_yddot, n_feature_y, FLAGS_rom_option);
  }

  RomData rom = RomData(n_y, n_tau, n_feature_y, n_feature_yddot, B_tau,
                        theta_y, theta_yddot);

  bool is_get_nominal = FLAGS_is_get_nominal;
  int max_inner_iter_pass_in = is_get_nominal ? 200 : max_inner_iter;

//  string init_file_pass_in = "";
  string init_file_pass_in = getInitFileName(dir, num, is_rerun,
      initial_guess_idx);
  int sample_idx = 0;
  string prefix = to_string(num) +  "_" + to_string(sample_idx) + "_";

  inner_loop_setting.n_node = (FLAGS_n_node>0)? FLAGS_n_node : 20;//fix number of nodes
  inner_loop_setting.max_iter = max_inner_iter_pass_in;
  inner_loop_setting.prefix = prefix;
  inner_loop_setting.init_file = init_file_pass_in;

  // Vectors/Matrices for the outer loop
  int N_sample = 1;
  SubQpData QPs(N_sample);
  // reset is_success_vec before trajectory optimization
  for (int i = 0; i < N_sample; i++) {
    *(QPs.is_success_vec[i]) = 0;
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
  trajOptGivenWeights(
      std::ref(plant),
      std::ref(plant_autoDiff),
      std::ref(rom),
      inner_loop_setting,
      task,
      std::ref(QPs),
      std::ref(thread_finished_vec),
      is_get_nominal,
      extend_model_this_iter,
      sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
      FLAGS_rom_option, FLAGS_robot_option);
}

//Save boundary point information
void SaveBoundaryPointInfor(const string dir,int boundary_point_index,
    int traj_num,const VectorXd& boundary_point){
  VectorXd boundary_point_infor(boundary_point.rows()+2);
  double boundary_point_cost = (readCSV(dir + to_string(traj_num) +
      string("_0_c.csv")))(0, 0);
  boundary_point_infor << traj_num,boundary_point_cost,boundary_point;
  writeCSV(dir + to_string(boundary_point_index) +  "_" +
      string("boundary_point.csv"), boundary_point_infor);
  cout << "boundary point index | stride length | ground incline"
          " | velocity | turning rate"<<endl;
  cout << " \t "<<boundary_point_index;
  for(int i=0;i<boundary_point.rows();i++)
  {
    cout<<"\t" <<" | "<<"\t"<<boundary_point[i];
  }
  cout<<endl;
}

//check the solution of trajectory optimization and rerun if necessary
void CheckSolution(const Task& task, const string dir,int traj_num){
  int rerun = 0;
  int max_rerun_num = 4;
  int is_success;
  //check if snopt find a solution successfully. If not, rerun the Traj Opt
  for(rerun=0;rerun<max_rerun_num;rerun++){
    is_success = (readCSV(dir + to_string(traj_num) +
        string("_0_is_success.csv")))(0, 0);
    if(is_success==0){
      TrajOptGivenModel(task, dir, traj_num,true);
    }
    else{
      break;
    }
  }
  //if snopt still can't find a solution, try to use adjacent sample to help
  for(rerun=0;rerun<max_rerun_num;rerun++){
    is_success = (readCSV(dir + to_string(traj_num) +
        string("_0_is_success.csv")))(0, 0);
    if(is_success==0){
      TrajOptGivenModel(task, dir, traj_num,true,traj_num-1);
    }
    else{
      break;
    }
  }
  //if snopt still failed to find a solution,turn off the scaling option
  // and try again
  for(rerun=0;rerun<max_rerun_num;rerun++){
    is_success = (readCSV(dir + to_string(traj_num) +
        string("_0_is_success.csv")))(0, 0);
    if(is_success==0){
      TrajOptGivenModel(task, dir, traj_num,true,-1,false);
    }
    else{
      break;
    }
  }
}

//search the boundary point along one direction
void BoundaryForOneDirection(const string dir,int max_iteration,
    const Task& initial_task,const VectorXd& step_direction, const VectorXd& step_size,
    double max_cost,int& traj_num,int& boundary_point_idx){
  int iter;
  Task task=initial_task;//create a copy for future use
  VectorXd new_task(task.dim());
  VectorXd last_task = Eigen::Map<const VectorXd>(task.get().data(),
      task.get().size());
  VectorXd boundary_point(task.dim());
  VectorXd step = step_size.array()*step_direction.array();
  MatrixXd cost_list;
  double decay_factor;//take a large step at the beginning
  cout << "sample# (rerun #) | stride | incline | velocity | turning rate | "
          "init_file | Status | Solve time | Cost (tau cost)\n";
  for (iter = 1; iter <= max_iteration; iter++){
    decay_factor = 2.5*pow(0.95,iter);
    //if decay_factor is larger than 1, use it to decrease the step size;
    //otherwise directly use the step size.
    if(decay_factor>1){
      new_task = last_task+decay_factor*step;
      last_task = new_task;
    }
    else{
      new_task = last_task+step;
      last_task = new_task;
    }
    //if stride length is negative or zero,stop searching
    if(new_task[0]<=0){
      boundary_point_idx += 1;
      boundary_point = new_task-step;
      SaveBoundaryPointInfor(dir,boundary_point_idx,
          traj_num,boundary_point);
      break;
    }
    //start searching
    //set tasks
    vector<double> new_task_vector(new_task.data(),
        new_task.data()+new_task.size());
    task.set(new_task_vector);
    //save tasks
    traj_num += 1;
    writeCSV(dir + to_string(traj_num) +
        string("_0_task.csv"), new_task);
    //run trajectory optimization and judge the solution
    TrajOptGivenModel(task, dir, traj_num, false);
    CheckSolution(task,dir,traj_num);
    double sample_cost =
        (readCSV(dir + to_string(traj_num) + string("_0_c.csv")))(0, 0);
    // without a good initial guess, the initial point is easily stuck in a local minimum
    // use the first sample to judge the solution of initial point
    if(iter==1){
      double initial_cost =
          (readCSV(dir + string("0_0_c.csv")))(0, 0);
      if(initial_cost>1.2*sample_cost){
        TrajOptGivenModel(initial_task, dir, 0,true,traj_num);
      }
    }
    //save the trajectory optimization index and corresponding cost for further use
    cost_list.conservativeResize(cost_list.rows()+1, 2);
    cost_list.row(cost_list.rows()-1)<<traj_num,sample_cost;

    if(sample_cost>max_cost){
      boundary_point_idx += 1;
      boundary_point = new_task-step;
      SaveBoundaryPointInfor(dir,boundary_point_idx,
                             traj_num,boundary_point);
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
      VectorXd task_to_rerun = readCSV(dir + to_string(traj_idx)
                                           + string("_0_task.csv"));
      vector<double> task_vector(task_to_rerun.data(),
          task_to_rerun.data()+task_to_rerun.size());
      task.set(task_vector);
      //choose the result of sample with lower cost as initial guess
      if(cost_list(iter-1,1)<cost_list(iter+1,1)){
        TrajOptGivenModel(task, dir, traj_idx,
                          true, traj_idx-1);
        //make sure this sample success
        CheckSolution(task,dir,traj_idx);
      }
      else{
        TrajOptGivenModel(task, dir, traj_idx,
                          true, traj_idx+1);
        //make sure this sample success
        CheckSolution(task,dir,traj_idx);
      }
      //update cost list
      cost_list(iter,1) = readCSV(dir + to_string(traj_idx)
                                      + string("_0_c.csv"))(0,0);
    }
  }
  writeCSV(dir + to_string(boundary_point_idx) +
    string("_searching_direction.csv"), step_direction);
  writeCSV(dir +  to_string(boundary_point_idx)  +
      string("_cost_list.csv"), cost_list);
  cout << "\nFinish checking the cost:\n";
}

//Decide Search Vector and serach the task space
void SearchTaskSpace(int index,const SearchSetting& search_setting,
    VectorXd& extend_direction,const string dir,int max_iter,
    const Task& task,const VectorXd& task_delta,double cost_threshold,
    int& traj_opt_num,int& boundary_sample_num){
  if(index<search_setting.task_dim())
  {
    int ele=0;
    for (ele=0;ele<search_setting.get_n_element(index);ele++){
      extend_direction[index] = search_setting.get_element(index,ele);
      SearchTaskSpace(index+1,search_setting,extend_direction,dir,max_iter,
      task,task_delta,cost_threshold,traj_opt_num,boundary_sample_num);
    }
  }
  else{
    //filter with infinity norm to avoid repetitive searching
    //search along the direction
    if ((extend_direction.lpNorm<Eigen::Infinity>()) >= 1) {
      cout << "Start searching along direction: [";
      for (int i =0;i<search_setting.task_dim();i++) {
        cout << extend_direction[i] << " \t ";
      }
      cout<< "]" << endl;
      BoundaryForOneDirection(dir,max_iter,
                              task, extend_direction, task_delta,
                              cost_threshold,traj_opt_num,boundary_sample_num);
    }
  }
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
   * cout basic program information
   */
  cout << "\nBasic information:\n";
  cout << FLAGS_program_name << endl;
  int robot_option = FLAGS_robot_option;
  int rom_option = FLAGS_rom_option;
  cout << "robot_option: " << robot_option << endl;
  cout << "rom_option: " << rom_option << endl;

  /*
   * initialize task space
   */
  cout << "\nInitialize task space:\n";
  Task task;//Traj Opt tasks
  SearchSetting search_setting;//Settings of searching the task space
  // create components for each dimension used to decide the direction
  // considering we can only visualize 2D landscape, we only search within
  // 2D space and fix other dimension
  vector<double> search_elements = {0,-1,-0.5,0.5,1};
  vector<double> non_search_elements = {0};
  if(robot_option==0)
  {
    vector<vector<double>> elements{
      FLAGS_search_sl ? search_elements : non_search_elements,
      FLAGS_search_gi ? search_elements : non_search_elements,
      FLAGS_search_v ? search_elements : non_search_elements};
    task = Task({"stride length", "ground incline",
                 "velocity"});
    search_setting = SearchSetting(3,{"stride length", "ground incline",
                                      "velocity"},{0.25,0,0.4},
                                          {0.01,0.01,0.02},elements);
  }
  else if(robot_option==1){
    vector<vector<double>> elements{
      FLAGS_search_sl ? search_elements : non_search_elements,
      FLAGS_search_gi ? search_elements : non_search_elements,
      FLAGS_search_v ? search_elements : non_search_elements,
      FLAGS_search_tr ? search_elements : non_search_elements};
    task = Task({"stride length", "ground incline",
                 "velocity", "turning rate"});
    search_setting = SearchSetting(4,{"stride length", "ground incline",
                                      "velocity","turning rate"},
                                   {0.3,0,0.5,0},{0.01,0.01,0.01,0.02},elements);
  }
  //cout initial point information
  int dim = 0;
  for(dim = 0;dim<search_setting.task_dim();dim++)
  {
    cout<<"initial "<<search_setting.names()[dim]<<
    ": "+to_string(search_setting.task_0()[dim])<<endl;
  }

  /*
   * Iteration setting
   */
  cout << "\nIteration setting:\n";
  cout<<"get nominal cost: "<<FLAGS_is_get_nominal<<endl;
  cout<<"use optimized model: "<<FLAGS_use_optimized_model<<endl;
  cout<<"optimized model index: "<<FLAGS_theta_index<<endl;
  int max_iter = FLAGS_max_outer_iter;
  //TODO:decide the threshold under different situation
  double cost_threshold = 30;
  if(robot_option==0)
  {
    if(FLAGS_is_get_nominal){
      cost_threshold = 35;
    }
    else{
      cost_threshold = 30;
    }
  }
  else if(robot_option==1){
    if(FLAGS_is_get_nominal){
      cost_threshold = 35;
    }
    else{
      cost_threshold = 30;
    }
  }
  cout<<"cost_threshold "<<cost_threshold<<endl;

  cout<<"search along: ";
  for (dim=0;dim<search_setting.task_dim();dim++)
  {
    if(search_setting.get_n_element(dim)>1){
      cout<<search_setting.names()[dim]<<" ";
    }
  }
  cout<<endl;

  /*
   * start iteration
   */
  int boundary_sample_num = 0;//use this to record the index of boundary point
  int traj_opt_num = 0;//use this to record the index of Traj Opt
  //evaluate initial point
  VectorXd initial_task = Eigen::Map<const VectorXd>
      (search_setting.task_0().data(),search_setting.task_0().size());
  task.set(search_setting.task_0());
  writeCSV(dir +  to_string(traj_opt_num)  +
      string("_0_task.csv"), initial_task);
  cout << "\nCalculate Central Point Cost:\n";
  cout << "sample# (rerun #) | stride | incline | velocity | turning rate | "
          "init_file | Status | Solve time | Cost (tau cost)\n";
  TrajOptGivenModel(task, dir, traj_opt_num, false);
  //make sure solution found for the initial point
  int init_is_success = (readCSV(dir + string("0_0_is_success.csv")))(0,0);
  while(!init_is_success){
    TrajOptGivenModel(task, dir, traj_opt_num, true);
    init_is_success = (readCSV(dir + string("0_0_is_success.csv")))(0,0);
  }

  // search the boundary
  VectorXd extend_direction(search_setting.task_dim());
  dim=0;
  VectorXd task_delta = Eigen::Map<const VectorXd>(
      search_setting.task_delta().data(),search_setting.task_delta().size());
  SearchTaskSpace(dim,search_setting,extend_direction,dir,max_iter,
      task,task_delta,cost_threshold,traj_opt_num,boundary_sample_num);
  return 0;
}
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::find_boundary(argc, argv);
}