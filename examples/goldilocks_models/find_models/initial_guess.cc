#include "examples/goldilocks_models/find_models/initial_guess.h"

using std::cout;
using std::endl;
using std::string;
using std::to_string;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib::goldilocks_models {
// edited by Jianshu to try a new way of setting initial guess

VectorXd GetThetaScale(const RomData& rom) {
  // considering the scale for theta doesn't have a significant impact on
  // improving the quality of the initial guess,set them all ones.
  return VectorXd::Ones(rom.n_y() + rom.n_yddot());
}

VectorXd GetGammaScale(const TasksGenerator* task_gen) {
  int gamma_dimension = task_gen->dim();
  VectorXd gamma_scale = VectorXd::Zero(gamma_dimension);
  // if not fixed task, we need to scale the gamma
  int dim = 0;
  double min;
  double max;
  for (dim = 0; dim < gamma_dimension; dim++) {
    min = task_gen->task_min(task_gen->names()[dim]);
    max = task_gen->task_min(task_gen->names()[dim]);
    if (!(min == max)) {
      // coefficient is different for different dimensions
      if (task_gen->names()[dim] == "turning_rate") {
        gamma_scale[dim] = 1.3 / (max - min);
      } else {
        gamma_scale[dim] = 1 / (max - min);
      }
    }
  }
  return gamma_scale;
}

// calculate the interpolation weight; update weight vector and solution matrix
void InterpolateAmongDifferentTasks(const string& dir, string prefix,
                                    const VectorXd& current_gamma,
                                    const VectorXd& gamma_scale,
                                    VectorXd& weight_vector,
                                    MatrixXd& solution_matrix) {
  // check if this sample is success
  int is_success = (readCSV(dir + prefix + string("_is_success.csv")))(0, 0);
  if (is_success == 1) {
    // extract past gamma
    VectorXd past_gamma = readCSV(dir + prefix + string("_task.csv"));
    // calculate the third power of L3 norm between two gammas
    VectorXd dif_gamma =
        (past_gamma - current_gamma).array().abs() * gamma_scale.array();
    VectorXd dif_gamma2 = dif_gamma.array().pow(2);
    double distance_gamma = (dif_gamma.transpose() * dif_gamma2)(0, 0);

    // extract the solution
    VectorXd w_to_interpolate = readCSV(dir + prefix + string("_w.csv"));
    // concatenate the weight and solution for further calculation
    solution_matrix.conservativeResize(w_to_interpolate.rows(),
                                       solution_matrix.cols() + 1);
    solution_matrix.col(solution_matrix.cols() - 1) = w_to_interpolate;
    weight_vector.conservativeResize(weight_vector.rows() + 1);
    weight_vector(weight_vector.rows() - 1) = 1 / distance_gamma;
  }
}

// calculate interpolated initial guess using weight vector and solution matrix
VectorXd CalculateInterpolation(VectorXd& weight_vector,
                                const MatrixXd& solution_matrix) {
  DRAKE_DEMAND(weight_vector.rows() > 0);
  // interpolation
  weight_vector = weight_vector/weight_vector.sum();
  VectorXd interpolated_solution = solution_matrix * weight_vector;
  return interpolated_solution;
}

string SetInitialGuessByInterpolation(const string& directory, int iter,
                                      int sample,
                                      const TasksGenerator* task_gen,
                                      const Task& task, const RomData& rom,
                                      bool use_database, int robot) {
  /* define some parameters used in interpolation
   * theta_range :decide the range of theta to use in interpolation
   * theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
   */
  double theta_range =
      0.004;  // this is tuned by robot_option=1,rom_option=2,3d task space
  int total_sample_num = task_gen->total_sample_number();

  VectorXd theta_scale = GetThetaScale(rom);
  VectorXd gamma_scale = GetGammaScale(task_gen);
  //    initialize variables used for setting initial guess
  VectorXd initial_guess;
  string initial_file_name;
  //    get theta of current iteration and task of current sample
  VectorXd current_theta = rom.theta();
  VectorXd current_gamma =
      Eigen::Map<const VectorXd>(task.get().data(), task.get().size());
  if (use_database && (iter < 2)) {
    string data_dir;
    // use solutions in database to calculate interpolated initial guess
    if (iter == 0) {
      // use nominal database
      data_dir = "../dairlib_data/goldilocks_models/database/robot_" +
                 to_string(robot) + "_nominal/";
    } else {
      // use initial database
      data_dir = "../dairlib_data/goldilocks_models/database/robot_" +
                 to_string(robot) + "_initial/";
    }
    // take out corresponding solution and store it in each column of w_gamma
    // calculate the interpolation weight and store it in weight_gamma
    MatrixXd w_gamma;
    VectorXd weight_gamma;

    // calculate the weighted sum of past solutions
    int sample_num = 0;
    string prefix;
    while (file_exist(data_dir + to_string(sample_num) + "_0"
         + string("_is_success.csv"))) {
      prefix = to_string(sample_num) + "_0";
      InterpolateAmongDifferentTasks(data_dir, prefix, current_gamma,
                                     gamma_scale, weight_gamma, w_gamma);
      sample_num = sample_num + 1;
    }
    initial_guess = CalculateInterpolation(weight_gamma, w_gamma);
    //    save initial guess and set init file
    initial_file_name = prefix + string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);
  } else {
    DRAKE_DEMAND(iter > 0);
    // There are two-stage interpolation here.
    // Get interpolated results using solutions of different tasks for each
    // theta. Then calculate interpolation using results from different theta.
    VectorXd weight_theta;  // each element corresponds to a weight
    MatrixXd w_theta;       // each column stores a interpolated result
    int past_iter;
    int sample_num;
    int iter_start;
    string prefix;
    //    For iter = 1, use the solutions from iteration 0 to set the initial
    //    guess
    if (iter == 1) {
      iter_start = 0;
    }
    // if iter > 1, use the results from past iterations to interpolate and set
    // the initial guess
    else {
      iter_start = 1;
    }
    for (past_iter = iter - 1; past_iter >= iter_start; past_iter--) {
      // find useful theta according to the difference between previous theta
      // and new theta
      VectorXd past_theta_s =
          readCSV(directory + to_string(past_iter) + string("_theta_y.csv"));
      VectorXd past_theta_sDDot = readCSV(directory + to_string(past_iter) +
                                          string("_theta_yddot.csv"));
      VectorXd past_theta(past_theta_s.rows() + past_theta_sDDot.rows());
      past_theta << past_theta_s, past_theta_sDDot;
      double theta_diff =
          (past_theta - current_theta).norm() / current_theta.norm();
      if ( (theta_diff < theta_range) ) {
        // take out corresponding solution and store it in each column of
        // w_gamma calculate the interpolation weight and store it in
        // weight_gamma
        MatrixXd w_gamma;
        VectorXd weight_gamma;
        // calculate the weighted sum of solutions from one iteration
        for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
          prefix = to_string(past_iter) + string("_") + to_string(sample_num);
          InterpolateAmongDifferentTasks(directory, prefix, current_gamma,
                                         gamma_scale, weight_gamma, w_gamma);
        }
        // calculate the weighted sum for this iteration
        VectorXd w_to_interpolate =
            CalculateInterpolation(weight_gamma, w_gamma);
        // calculate the weight for the result above using the difference
        // between past theta and current theta
        VectorXd dif_theta =
            (past_theta - current_theta).array().abs() * theta_scale.array();
        double distance_theta = (dif_theta.transpose() * dif_theta)(0, 0);
        // if theta in this iteration accidentally equals to current theta, no
        // need to interpolate and just use the solution from this iteration.
        if (distance_theta == 0) {
          w_theta.conservativeResize(w_to_interpolate.rows(), 1);
          w_theta << w_to_interpolate;
          weight_theta.conservativeResize(1);
          weight_theta << 1;
          break;
        }
        // else concatenate the weighted sum of this iteration and the weight
        // for it
        else {
          w_theta.conservativeResize(w_to_interpolate.rows(),
                                     w_theta.cols() + 1);
          w_theta.col(w_theta.cols() - 1) = w_to_interpolate;
          weight_theta.conservativeResize(weight_theta.rows() + 1);
          weight_theta(weight_theta.rows() - 1) = 1 / distance_theta;
        }
      }
    }
    initial_guess = CalculateInterpolation(weight_theta, w_theta);
    //    save initial guess and set init file
    initial_file_name = to_string(iter) + "_" + to_string(sample) +
                        string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);
  }
  return initial_file_name;
}

}  // namespace dairlib::goldilocks_models