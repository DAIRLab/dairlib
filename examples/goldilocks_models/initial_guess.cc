//
// Created by jianshu on 3/25/20.
//
#include "examples/goldilocks_models/initial_guess.h"

namespace dairlib::goldilocks_models {
//edited by Jianshu to try a new way of setting initial guess

MatrixXd get_theta_scale(const string directory, int iter){
    VectorXd current_theta_s = readCSV(directory + to_string(iter) + string("_theta_s.csv"));
    VectorXd current_theta_sDDot = readCSV(directory + to_string(iter) + string("_theta_sDDot.csv"));
    VectorXd theta_scale = 1*VectorXd::Ones(current_theta_s.rows()+current_theta_sDDot.rows());
    return theta_scale;
}

MatrixXd get_gamma_scale(GridTasksGenerator task_gen){
  int gamma_dimension = task_gen.dim();
  VectorXd gamma_scale = VectorXd::Zero(gamma_dimension);
  // if not fixed task, we need to scale the gamma
  int dim = 0;
  double min;
  double max;
  for (dim=0;dim<gamma_dimension;dim++)
  {
    min = task_gen.task_min(task_gen.names()[dim]);
    max = task_gen.task_min(task_gen.names()[dim]);
    if(! (min == max))
    {
      //coefficient is different for different dimensions
      if(task_gen.names()[dim]=="turning_rate")
      {
        gamma_scale[dim] = 1.3/(max-min);
      }
      else
      {
        gamma_scale[dim] = 1/(max-min);
      }
    }
  }
  return gamma_scale;
}

string set_initial_guess(const string directory, int iter, int sample,
                         GridTasksGenerator task_gen, bool use_database,int robot) {
  /* define some parameters used in interpolation
* theta_range :decide the range of theta to use in interpolation
* theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
*/
  double theta_range = 0.004;//this is tuned by robot_option=1,rom_option=2,3d task space
  int gamma_dimension = task_gen.dim();
  int total_sample_num = task_gen.total_sample_number();

  MatrixXd theta_scale = get_theta_scale(directory, iter);
  MatrixXd gamma_scale = get_gamma_scale(task_gen);
//    initialize variables used for setting initial guess
  VectorXd initial_guess;
  string initial_file_name;
  VectorXd weight_theta;
  MatrixXd w_theta;
  int past_iter;
  int sample_num;
//    get theta of current iteration and task of current sample
  VectorXd current_theta_s = readCSV(directory + to_string(iter) + string("_theta_y.csv"));
  VectorXd current_theta_sDDot = readCSV(directory + to_string(iter) + string("_theta_yDDot.csv"));
  VectorXd current_theta(current_theta_s.rows()+current_theta_sDDot.rows());
  current_theta << current_theta_s,current_theta_sDDot;
  VectorXd current_gamma = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
      + string("_task.csv"));
  int iter_start;
  string data_dir;
  //if use solutions from database
  if(use_database && (iter<2)){
    if(iter==0){
      //use nominal database
      data_dir = "../dairlib_data/goldilocks_models/database/robot_" +
          to_string(robot) + "_nominal/";
    }
    else{
      //use initial database
      data_dir = "../dairlib_data/goldilocks_models/database/robot_" +
          to_string(robot) + "_initial/";
    }
    //use solutions in database to calculate interpolated initial guess
    VectorXd initial_guess;
    //take out corresponding w and calculate the weight for interpolation
    MatrixXd w_gamma;
    VectorXd weight_gamma;

    //calculate the weighted sum of past solutions
    int sample_num = 0;
    while(file_exist(data_dir + to_string(sample_num)+string("_0_is_success.csv"))){
      //check if this sample is success
      int is_success = (readCSV(data_dir + to_string(sample_num)
                                    + string("_0_is_success.csv")))(0,0);
      if(is_success == 1) {
        //extract past gamma
        VectorXd past_gamma = readCSV(data_dir + to_string(sample_num)
                                          + string("_0_gamma.csv"));
        //calculate the weight for each sample using the third power of the difference between gamma
        VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*gamma_scale.array();
        VectorXd dif_gamma2 = dif_gamma.array().pow(2);
        double distance_gamma =  (dif_gamma.transpose() * dif_gamma2)(0,0);
//        //extract the solution of this sample
        VectorXd w_to_interpolate = readCSV(data_dir + to_string(sample_num)
                                                + string("_0_w.csv"));
        //concatenate the weight and solution for further calculation
        w_gamma.conservativeResize(w_to_interpolate.rows(),w_gamma.cols()+1);
        w_gamma.col(w_gamma.cols()-1) = w_to_interpolate;
        weight_gamma.conservativeResize(weight_gamma.rows()+1);
        weight_gamma(weight_gamma.rows()-1)= 1 /distance_gamma;
      }
      sample_num = sample_num+1;
    }
    DRAKE_DEMAND(weight_gamma.rows()>0);
    //    normalize weight
    weight_gamma = weight_gamma / weight_gamma.sum();
    initial_guess = w_gamma * weight_gamma;
    //    save initial guess and set init file
    initial_file_name = to_string(iter) + "_" + to_string(sample)
        + string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);

  }
  else {
    //    For iter = 1, use the solutions from iteration 0 to set the initial guess
    if (iter == 1) {
      iter_start = 0;
    }
      //if iter > 1, use the results from past iterations to interpolate and set the initial guess
    else {
      iter_start = 1;
    }
    for (past_iter = iter - 1; past_iter >= iter_start; past_iter--) {
      //find useful theta according to the difference between previous theta and new theta
      VectorXd past_theta_s =
          readCSV(directory + to_string(past_iter) + string("_theta_y.csv"));
      VectorXd past_theta_sDDot = readCSV(
          directory + to_string(past_iter) + string("_theta_yDDot.csv"));
      VectorXd past_theta(past_theta_s.rows() + past_theta_sDDot.rows());
      past_theta << past_theta_s, past_theta_sDDot;
      double theta_diff =
          (past_theta - current_theta).norm() / current_theta.norm();
      if ((theta_diff < theta_range)) {
        //take out corresponding w and calculate the weight for interpolation
        MatrixXd w_gamma;
        VectorXd weight_gamma;
        //calculate the weighted sum of solutions from one iteration
        for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
          //check if this sample is success
          int is_success =
              (readCSV(directory + to_string(past_iter) + string("_")
                           + to_string(sample_num)
                           + string("_is_success.csv")))(0, 0);
          if (is_success == 1) {
            //extract past gamma
            VectorXd past_gamma = readCSV(directory + to_string(past_iter) + string("_") + to_string(sample)
              + string("_task.csv"));
            //calculate the weight for each sample using the 3-norm of the difference between gamma
            VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()
                * gamma_scale.array();
            VectorXd dif_gamma2 = dif_gamma.array().pow(2);
            double distance_gamma = (dif_gamma.transpose() * dif_gamma2)(0, 0);
            //extract the solution of this sample
            VectorXd w_to_interpolate =
                readCSV(directory + to_string(past_iter) + string("_")
                            + to_string(sample_num) + string("_w.csv"));
            // if this sample accidentally have the same gamma with current sample, no need to interpolate and
            // just use the solution from this sample.
            if (distance_gamma == 0) {
              w_gamma.conservativeResize(w_to_interpolate.rows(), 1);
              w_gamma << w_to_interpolate;
              weight_gamma.conservativeResize(1);
              weight_gamma << 1;
              break;
            }
              //else concatenate the weight and solution for further calculation
            else {
              w_gamma.conservativeResize(w_to_interpolate.rows(),
                                         w_gamma.cols() + 1);
              w_gamma.col(w_gamma.cols() - 1) = w_to_interpolate;
              weight_gamma.conservativeResize(weight_gamma.rows() + 1);
              weight_gamma(weight_gamma.rows() - 1) = 1 / distance_gamma;
            }
          }
        }
        //calculate the weighted sum for this iteration
        weight_gamma = weight_gamma / weight_gamma.sum();
        VectorXd w_to_interpolate = w_gamma * weight_gamma;
        //calculate the weight for the result above using the difference between past theta and current theta
        VectorXd dif_theta =
            (past_theta - current_theta).array().abs() * theta_scale.array();
        double distance_theta = (dif_theta.transpose() * dif_theta)(0, 0);
        // if theta in this iteration accidentally equals to current theta, no need to interpolate and
        // just use the solution from this iteration.
        if (distance_theta == 0) {
          w_theta.conservativeResize(w_to_interpolate.rows(), 1);
          w_theta << w_to_interpolate;
          weight_theta.conservativeResize(1);
          weight_theta << 1;
          break;
        }
          // else concatenate the weighted sum of this iteration and the weight for it
        else {
          w_theta.conservativeResize(w_to_interpolate.rows(),
                                     w_theta.cols() + 1);
          w_theta.col(w_theta.cols() - 1) = w_to_interpolate;
          weight_theta.conservativeResize(weight_theta.rows() + 1);
          weight_theta(weight_theta.rows() - 1) = 1 / distance_theta;
        }
      }
    }
//  debug for no appropriate theta for interpolation. Wrong setting of theta range
    DRAKE_DEMAND(weight_theta.rows() > 0);
//    normalize weight
    weight_theta = weight_theta / weight_theta.sum();
    initial_guess = w_theta * weight_theta;
//    save initial guess and set init file
    initial_file_name = to_string(iter) + "_" + to_string(sample)
        + string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);

  }
  return initial_file_name;
}

int test_initial_guess(int iter_,int sample_,int robot_){
  //create test data and save it
  int iter = iter_;
  int sample = sample_;
  int robot = robot_;
  int use_database = false;
  GridTasksGenerator task_gen;
  if (robot == 0) {
    task_gen = GridTasksGenerator(
        3, {"stride length", "ground incline", "velocity"},
        {10, 5, 5}, {0.25, 0, 0.4},
        {0.015, 0.05, 0.02}, true);
  } else{
    task_gen = GridTasksGenerator(
        4, {"stride length", "ground incline", "velocity", "turning rate"},
        {10, 5, 5, 5},
        {0.3, 0, 0.5, 0}, {0.015, 0.05, 0.04, 0.125}, true);
  }
  int total_sample_num = task_gen.total_sample_number();
  const string dir = "../dairlib_data/goldilocks_models/find_models/robot_1_test/";
  if (!CreateFolderIfNotExist(dir)) return 0;

  bool use_created_data = true;
  if(use_created_data) {
    //for each iteration, create theta_s and theta_sDDot
    int iteration = 0;
    for (iteration = 0; iteration <= iter; iteration++) {
      VectorXd theta_y = VectorXd::Random(70);
      VectorXd theta_yDDot = VectorXd::Random(7);
      writeCSV(dir + to_string(iteration) + string("_theta_y.csv"),
               theta_y);
      writeCSV(dir + to_string(iteration) + string("_theta_yDDot.csv"),
               theta_yDDot);
      //for each sample, create gamma, is_success and w
      int sample = 0;
      int dim = 0;
      for (sample = 0; sample <= total_sample_num; sample++) {
        string prefix = to_string(iteration) + "_" + to_string(sample) + "_";
        VectorXd gamma(task_gen.dim());
        for (dim=0;dim<task_gen.dim();dim++)
        {
          double min = task_gen.task_min(task_gen.names()[dim]);
          double max = task_gen.task_min(task_gen.names()[dim]);
          std::uniform_real_distribution<double> dist(min,max);
          std::default_random_engine re;
          gamma[dim] = dist(re);
        }
        writeCSV(dir + prefix + string("_task.csv"),gamma);
        bool is_success = 1;
        writeCSV(dir + prefix + string("is_success.csv"),
                 is_success * MatrixXd::Ones(1, 1));
        VectorXd w = VectorXd::Random(1478);
        writeCSV(dir + prefix + string("w.csv"), w);
      }
    }
  }
  string initial_file = set_initial_guess(dir, iter, sample,task_gen,
                                          use_database,robot);
  return 0;
}

} //namespace