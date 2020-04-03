//
// Created by jianshu on 3/25/20.
//
#include "examples/goldilocks_models/initial_guess.h"

namespace dairlib::goldilocks_models {
//edited by Jianshu to try a new way of setting initial guess
string set_initial_guess(const string directory, int iter, int sample, int total_sample_num,
                         double min_sl, double max_sl, double min_gi, double max_gi) {
/* define some parameters used in interpolation
* theta_range :decide the range of theta to use in interpolation
* theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
*/
    double theta_range = 0.01;
    double theta_scale;
    double gamma_scale = pow(max_sl-min_sl,2)+pow(max_gi-min_gi,2);
    int gamma_dimension = 2;
//    initialize variables used for setting initial guess
    VectorXd initial_guess;
    VectorXd weight;
    MatrixXd w_near;
    int past_iter;
    int sample_num;
//    get theta of current iteration and task of current sample
    VectorXd current_theta = readCSV(directory + to_string(iter) + string("_theta_s.csv"));
    MatrixXd current_ground_incline = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                              + string("_ground_incline.csv"));
    MatrixXd current_stride_length = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                             + string("_stride_length.csv"));
    VectorXd current_gamma(gamma_dimension);
    current_gamma << current_ground_incline(0, 0), current_stride_length(0, 0);
//    get initial theta and set theta scale
    VectorXd initial_theta = readCSV(directory + to_string(0) + string("_theta_s.csv"));
    theta_scale = (initial_theta-current_theta).squaredNorm();

    for (past_iter = iter - 1; past_iter >= 0; past_iter--) {
//        find useful theta according to the difference between previous theta and new theta
        VectorXd past_theta = readCSV(directory + to_string(past_iter) + string("_theta_s.csv"));
        double theta_diff = (past_theta - current_theta).norm() / current_theta.norm();
        if (theta_diff < theta_range) {
//            take out corresponding w and calculate the weight for interpolation
            for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
                MatrixXd past_ground_incline = readCSV(directory + to_string(past_iter) + string("_")
                                                       + to_string(sample_num) + string("_ground_incline.csv"));
                MatrixXd past_stride_length = readCSV(directory + to_string(past_iter) + string("_")
                                                      + to_string(sample_num) + string("_stride_length.csv"));
                VectorXd past_gamma(gamma_dimension);
                past_gamma << past_ground_incline(0, 0), past_stride_length(0, 0);
                double distance = ((past_theta - current_theta) / theta_scale).squaredNorm()
                                  + ((past_gamma - current_gamma) / gamma_scale).squaredNorm();
//                debug for getting wrong weight
                if (distance == 0) {
                    return to_string(iter - 1) + "_" + to_string(sample) + string("_w.csv");
                }
                weight.conservativeResize(weight.rows() + 1);
                weight(weight.rows() - 1) = 1 / sqrt(distance);
                VectorXd w_to_interpolate = readCSV(directory + to_string(past_iter) + string("_")
                                                    + to_string(sample_num) + string("_w.csv"));
                w_near.conservativeResize(w_to_interpolate.rows(), w_near.cols() + 1);
                w_near.col(w_near.cols() - 1) = w_to_interpolate;
            }
        } else {
            break;
        }
    }
//    debug for no appropriate theta for interpolation
    if (weight.rows() == 0) {
        cout << "wrong setting of theta range";
        return to_string(iter - 1) + "_" + to_string(sample) + string("_w.csv");
    }
//    normalize weight
    weight = weight / weight.sum();
    initial_guess = w_near * weight;
//    save initial guess and set init file
    string initial_file_name = to_string(iter) + "_" + to_string(sample) + string("_initial_guess.csv");
    writeCSV(directory + initial_file_name, initial_guess);

    return initial_file_name;
}

} //namespace