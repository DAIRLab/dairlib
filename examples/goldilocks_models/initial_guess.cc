//
// Created by jianshu on 3/25/20.
//
#include "examples/goldilocks_models/initial_guess.h"

namespace dairlib::goldilocks_models {
//edited by Jianshu to try a new way of setting initial guess

MatrixXd get_theta_scale(const string directory, int iter){
//    VectorXd initial_theta = readCSV(directory + to_string(0) + string("_theta_s.csv"));
    VectorXd current_theta = readCSV(directory + to_string(iter) + string("_theta_s.csv"));
    VectorXd theta_scale = 1*VectorXd::Ones(current_theta.rows());
    MatrixXd diag_theta_scale = theta_scale.asDiagonal();
    return diag_theta_scale;
}

MatrixXd get_gamma_scale(int gamma_length, double min_sl, double max_sl, double min_gi, double max_gi,
        double min_tr, double max_tr){
    VectorXd gamma_scale(gamma_length);
    gamma_scale << 1, 1, 2;
    MatrixXd diag_gamma_scale = gamma_scale.asDiagonal();
    return diag_gamma_scale;
}

string set_initial_guess(const string directory, int iter, int sample, int total_sample_num,
                         double min_sl, double max_sl, double min_gi, double max_gi, double min_tr,
                         double max_tr) {
    /* define some parameters used in interpolation
* theta_range :decide the range of theta to use in interpolation
* theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
*/
    double theta_range = 1; //Have not tuned yet!!
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
    VectorXd current_theta = readCSV(directory + to_string(iter) + string("_theta_s.csv"));
    MatrixXd current_ground_incline = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                              + string("_ground_incline.csv"));
    MatrixXd current_stride_length = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                             + string("_stride_length.csv"));
    MatrixXd current_turning_rate = readCSV(directory + to_string(iter) + string("_") + to_string(sample)
                                             + string("_turning_rate.csv"));
    VectorXd current_gamma(gamma_dimension);
    current_gamma << current_ground_incline(0, 0), current_stride_length(0, 0), current_turning_rate(0,0);
//    For iter = 1, theta_1 is the same with theta_0. So use the results of iter_0.
    if(iter == 1)
    {
        double distance_gamma_min = 100000; //initialize with a large one
        int sample_near = 0;
        for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
//                check if this sample is success
            int is_success = (readCSV(directory + to_string(iter-1) + string("_")
                                      + to_string(sample_num) + string("_is_success.csv")))(0,0);
            if(is_success == 1) {
                MatrixXd past_ground_incline = readCSV(directory + to_string(iter-1) + string("_")
                        +to_string(sample_num) + string("_ground_incline.csv"));
                MatrixXd past_stride_length = readCSV(directory + to_string(iter-1) + string("_")
                        + to_string(sample_num) + string("_stride_length.csv"));
                MatrixXd past_turning_rate = readCSV(directory + to_string(iter-1) + string("_")
                        + to_string(sample_num) + string("_turning_rate.csv"));
                VectorXd past_gamma(gamma_dimension);
                past_gamma << past_ground_incline(0, 0), past_stride_length(0, 0), past_turning_rate(0,0);

                // normalize gamma
                VectorXd normalize_scale = VectorXd::Zero(gamma_dimension);
                if(! (min_gi==max_gi))
                {
                    normalize_scale[0] = 1/(max_gi-min_gi);
                }
                if(! (min_sl==max_sl))
                {
                    normalize_scale[1] = 1/(max_sl-min_sl);
                }
                if(! (min_tr==max_tr))
                {
                    normalize_scale[2] = 1/(max_tr-min_tr);
                }

                VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*normalize_scale.array();
                VectorXd dif_gamma2 = dif_gamma.array().pow(2);
                double distance_gamma =  (dif_gamma.transpose() * gamma_scale * dif_gamma2)(0,0);
                if (distance_gamma < distance_gamma_min) {
                    distance_gamma_min = distance_gamma;
                    sample_near = sample_num;
                }
            }
        }
        initial_guess = readCSV(directory + to_string(iter-1) + string("_")
                                            + to_string(sample_near) + string("_w.csv"));
        string initial_file_name = to_string(iter) + "_" + to_string(sample) + string("_initial_guess.csv");
        writeCSV(directory + initial_file_name, initial_guess);

        return initial_file_name;
    }
    else
    {
    for (past_iter = iter - 1; past_iter > 0; past_iter--) {
//        find useful theta according to the difference between previous theta and new theta
        VectorXd past_theta = readCSV(directory + to_string(past_iter) + string("_theta_s.csv"));
        double theta_diff = (past_theta - current_theta).norm() / current_theta.norm();
        if ((theta_diff < theta_range) && (iter-past_iter<5)) {
//            take out corresponding w and calculate the weight for interpolation
            MatrixXd w_gamma;
            VectorXd weight_gamma;
            for (sample_num = 0; sample_num < total_sample_num; sample_num++) {
//                check if this sample is success
                int is_success = (readCSV(directory + to_string(past_iter) + string("_")
                        + to_string(sample_num) + string("_is_success.csv")))(0,0);
                if(is_success == 1) {
                    MatrixXd past_ground_incline = readCSV(directory + to_string(past_iter) + string("_")
                            + to_string(sample_num) + string("_ground_incline.csv"));
                    MatrixXd past_stride_length = readCSV(directory + to_string(past_iter) + string("_")
                            + to_string(sample_num) + string("_stride_length.csv"));
                    MatrixXd past_turning_rate = readCSV(directory + to_string(past_iter) + string("_")
                            + to_string(sample_num) + string("_turning_rate.csv"));
                    VectorXd past_gamma(gamma_dimension);
                    past_gamma << past_ground_incline(0, 0), past_stride_length(0, 0), past_turning_rate(0, 0);
                    // normalize gamma
                    VectorXd normalize_scale = VectorXd::Zero(gamma_dimension);
                    if(! (min_gi==max_gi))
                    {
                        normalize_scale[0] = 1/(max_gi-min_gi);
                    }
                    if(! (min_sl==max_sl))
                    {
                        normalize_scale[1] = 1/(max_sl-min_sl);
                    }
                    if(! (min_tr==max_tr))
                    {
                        normalize_scale[2] = 1/(max_tr-min_tr);
                    }
                    VectorXd dif_gamma = (past_gamma - current_gamma).array().abs()*normalize_scale.array();
                    VectorXd dif_gamma2 = dif_gamma.array().pow(2);
                    double distance_gamma =  (dif_gamma.transpose() * gamma_scale * dif_gamma2)(0,0);
                    VectorXd w_to_interpolate = readCSV(directory + to_string(past_iter) + string("_")
                                                        + to_string(sample_num) + string("_w.csv"));
                    if (distance_gamma == 0) {
                        w_gamma.conservativeResize(w_to_interpolate.rows(), 1);
                        w_gamma << w_to_interpolate;
                        weight_gamma.conservativeResize(1);
                        weight_gamma << 1;
                        break;
                    }
                    else{
                        w_gamma.conservativeResize(w_to_interpolate.rows(),w_gamma.cols()+1);
                        w_gamma.col(w_gamma.cols()-1) = w_to_interpolate;
                        weight_gamma.conservativeResize(weight_gamma.rows()+1);
                        weight_gamma(weight_gamma.rows()-1)= 1 /distance_gamma;
                    }
                }
            }
            double distance_theta =  ((past_theta - current_theta).transpose() * theta_scale *
                                     (past_theta - current_theta))(0,0);
            weight_gamma = weight_gamma/weight_gamma.sum();
            VectorXd w_to_interpolate = w_gamma*weight_gamma;
            if (distance_theta == 0) {
                w_theta.conservativeResize(w_to_interpolate.rows(), 1);
                w_theta << w_to_interpolate;
                weight_theta.conservativeResize(1);
                weight_theta << 1;
                break;
            }
            else{
                w_theta.conservativeResize(w_to_interpolate.rows(),w_theta.cols()+1);
                w_theta.col(w_theta.cols()-1) = w_to_interpolate;
                weight_theta.conservativeResize(weight_theta.rows()+1);
                weight_theta(weight_theta.rows()-1)= 1 /distance_theta;
            }
        }

    }
//    debug for no appropriate theta for interpolation
    if (weight_theta.rows() == 0) {
        cout << "wrong setting of theta range,no appropriate w to interpolate";
        return to_string(iter - 1) + "_" + to_string(sample) + string("_w.csv");
    }
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