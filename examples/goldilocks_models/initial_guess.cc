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

MatrixXd get_gamma_scale(int gamma_length, double min_sl, double max_sl, double min_gi, double max_gi,
        double min_tr, double max_tr){
  VectorXd gamma_scale = VectorXd::Zero(gamma_length);
  // if not fixed task, we need to scale the gamma
  if(! (min_gi==max_gi))
  {
    gamma_scale[0] = 1/(max_gi-min_gi);
  }
  if(! (min_sl==max_sl))
  {
    gamma_scale[1] = 1/(max_sl-min_sl);
  }
  if(! (min_tr==max_tr))
  {
    gamma_scale[2] = 1.3/(max_tr-min_tr);
  }
  return gamma_scale;
}

string set_initial_guess(const string directory, int iter, int sample, int total_sample_num,
                         double min_sl, double max_sl, double min_gi, double max_gi, double min_tr,
                         double max_tr) {
  /* define some parameters used in interpolation
* theta_range :decide the range of theta to use in interpolation
* theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
*/
  double theta_range = 0.004;//this is tuned by robot_option=1,rom_option=2,3d task space
  int gamma_dimension = 3;
  MatrixXd theta_scale = get_theta_scale(directory, iter);
  MatrixXd gamma_scale = get_gamma_scale(gamma_dimension, min_sl, max_sl, min_gi, max_gi, min_tr, max_tr);
//    initialize variables used for setting initial guess
  VectorXd initial_guess;
  VectorXd weight_theta;
  MatrixXd w_theta;
  int past_iter;
  int sample_num;
//    get theta of current iteration and task of current sample
  VectorXd current_theta_s = readCSV(directory + to_string(iter) + string("_theta_s.csv"));
  VectorXd current_theta_sDDot = readCSV(directory + to_string(iter) + string("_theta_sDDot.csv"));
  VectorXd current_theta(current_theta_s.rows()+current_theta_sDDot.rows());
  current_theta << current_theta_s,current_theta_sDDot;
  MatrixXd current_ground_incline = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                            + string("_ground_incline.csv"));
  MatrixXd current_stride_length = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                           + string("_stride_length.csv"));
  MatrixXd current_turning_rate = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                           + string("_turning_rate.csv"));
  VectorXd current_gamma(gamma_dimension);
  current_gamma << current_ground_incline(0, 0), current_stride_length(0, 0), current_turning_rate(0,0);
//    For iter = 1, we look for the closest task sample from iter 0 and use the solution from the sample
//    for initial guess
  if(iter == 1)
  {
    double distance_gamma_min = std::numeric_limits<double>::infinity(); //initialize with a large one
    int sample_near = 0;
    for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
      //check if this sample is success
      int is_success = (readCSV(directory + to_string(iter-1) + string("_")
                                + to_string(sample_num) + string("_is_success.csv")))(0,0);
      if(is_success == 1) {
        //extract past gamma
        MatrixXd past_ground_incline = readCSV(directory + to_string(iter-1) + string("_")
                +to_string(sample_num) + string("_ground_incline.csv"));
        MatrixXd past_stride_length = readCSV(directory + to_string(iter-1) + string("_")
                + to_string(sample_num) + string("_stride_length.csv"));
        MatrixXd past_turning_rate = readCSV(directory + to_string(iter-1) + string("_")
                + to_string(sample_num) + string("_turning_rate.csv"));
        VectorXd past_gamma(gamma_dimension);
        past_gamma << past_ground_incline(0, 0), past_stride_length(0, 0), past_turning_rate(0,0);
        // use the third power of the norm of difference between past gamma and current gamma to judge distance
        VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*gamma_scale.array();
        VectorXd dif_gamma2 = dif_gamma.array().pow(2);
        double distance_gamma =  (dif_gamma.transpose() * dif_gamma2)(0,0);
        if (distance_gamma < distance_gamma_min) {
            distance_gamma_min = distance_gamma;
            sample_near = sample_num;
        }
      }
    }
    //set the solution from the nearest sample as the initial guess
    initial_guess = readCSV(directory + to_string(iter-1) + string("_")
                                        + to_string(sample_near) + string("_w.csv"));
    string initial_file_name = to_string(iter) + "_" + to_string(sample) + string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);

    return initial_file_name;
  }
  //if iter > 1, use the results from past iterations to interpolate and set the initial guess
  else
  {
  for (past_iter = iter - 1; past_iter > 0; past_iter--) {
    //find useful theta according to the difference between previous theta and new theta
    VectorXd past_theta_s = readCSV(directory + to_string(past_iter) + string("_theta_s.csv"));
    VectorXd past_theta_sDDot = readCSV(directory + to_string(past_iter) + string("_theta_sDDot.csv"));
    VectorXd past_theta(past_theta_s.rows()+past_theta_sDDot.rows());
    past_theta << past_theta_s,past_theta_sDDot;
    double theta_diff = (past_theta - current_theta).norm() / current_theta.norm();
    if ((theta_diff < theta_range)) {
      //take out corresponding w and calculate the weight for interpolation
      MatrixXd w_gamma;
      VectorXd weight_gamma;
      //calculate the weighted sum of solutions from one iteration
      for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
        //check if this sample is success
        int is_success = (readCSV(directory + to_string(past_iter) + string("_")
                + to_string(sample_num) + string("_is_success.csv")))(0,0);
        if(is_success == 1) {
          //extract past gamma
          MatrixXd past_ground_incline = readCSV(directory + to_string(past_iter) + string("_")
                  + to_string(sample_num) + string("_ground_incline.csv"));
          MatrixXd past_stride_length = readCSV(directory + to_string(past_iter) + string("_")
                  + to_string(sample_num) + string("_stride_length.csv"));
          MatrixXd past_turning_rate = readCSV(directory + to_string(past_iter) + string("_")
                  + to_string(sample_num) + string("_turning_rate.csv"));
          VectorXd past_gamma(gamma_dimension);
          past_gamma << past_ground_incline(0, 0), past_stride_length(0, 0), past_turning_rate(0, 0);
          //calculate the weight for each sample using the third power of the difference between gamma
          VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*gamma_scale.array();
          VectorXd dif_gamma2 = dif_gamma.array().pow(2);
          double distance_gamma =  (dif_gamma.transpose() * dif_gamma2)(0,0);
          //extract the solution of this sample
          VectorXd w_to_interpolate = readCSV(directory + to_string(past_iter) + string("_")
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
          else{
              w_gamma.conservativeResize(w_to_interpolate.rows(),w_gamma.cols()+1);
              w_gamma.col(w_gamma.cols()-1) = w_to_interpolate;
              weight_gamma.conservativeResize(weight_gamma.rows()+1);
              weight_gamma(weight_gamma.rows()-1)= 1 /distance_gamma;
          }
        }
      }
      //calculate the weighted sum for this iteration
      weight_gamma = weight_gamma/weight_gamma.sum();
      VectorXd w_to_interpolate = w_gamma*weight_gamma;
      //calculate the weight for the result above using the difference between past theta and current theta
      VectorXd dif_theta = (past_theta - current_theta).array().abs()*theta_scale.array();
      double distance_theta =  (dif_theta.transpose() * dif_theta)(0,0);
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
      else{
          w_theta.conservativeResize(w_to_interpolate.rows(),w_theta.cols()+1);
          w_theta.col(w_theta.cols()-1) = w_to_interpolate;
          weight_theta.conservativeResize(weight_theta.rows()+1);
          weight_theta(weight_theta.rows()-1)= 1 /distance_theta;
      }
    }
  }
//  debug for no appropriate theta for interpolation. Wrong setting of theta range
  DRAKE_DEMAND(weight_theta.rows()>0);
//    normalize weight
  weight_theta = weight_theta / weight_theta.sum();
  initial_guess = w_theta * weight_theta;
//    save initial guess and set init file
  string initial_file_name = to_string(iter) + "_" + to_string(sample) + string("_initial_guess.csv");
  writeCSV(directory + initial_file_name, initial_guess);

  return initial_file_name;
  }
}

} //namespace