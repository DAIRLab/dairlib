//
// Created by jianshu on 3/16/20.
//
#include "examples/goldilocks_models/set_initial_guess.h"

//intialize the class before optimization
set_initial_guess::set_initial_guess(VectorXd& initial_theta,int samples_number) {
    theta = MatrixXd(initial_theta.rows(),1);
    theta.col(0) = initial_theta;
    sample_num = samples_number;
}
//add result of gradient descent
void set_initial_guess::add_theta(VectorXd& result_theta){
    theta.conservativeResize(theta.rows(),theta.cols()+1);
    theta.col(theta.cols()-1) = result_theta;
}
//add samples tasks gamma
void set_initial_guess::add_gamma(VectorXd& sample_gamma){
    gamma.conservativeResize(gamma.rows(),gamma.cols()+1);
    gamma.col(gamma.cols()-1) = sample_gamma;
}
//add trajectory optimization result w
void set_initial_guess::add_w(VectorXd& result_w){
    w.conservativeResize(w.rows(),w.cols()+1);
    w.col(w.cols()-1) = result_w;
}

//set initial guess for next iteration
VectorXd set_initial_guess::predict(VectorXd& new_theta, VectorXd& new_gamma){
    VectorXd initial_guess = VectorXd::Zero(w.rows());
    VectorXd weight;
    MatrixXd w_near;
    int i;
    int j;
    for(i=theta.cols()-1;i>=0;i--) {
//        find useful theta according to the difference between previous theta and new theta
        double theta_diff = (theta.col(i) - new_theta).norm() / new_theta.norm();
        if (theta_diff < theta_range) {
//            take out corresponding w and calculate the weight for interpolation
            for (j = 0; j < sample_num; j++) {
                double distance = ((theta.col(i) - new_theta)/theta_scale).squaredNorm()
                        + ((gamma.col(i*sample_num+j) - new_theta)/gamma_scale).squaredNorm();
                weight.conservativeResize(weight.rows()+1);
                weight(weight.rows()-1) = 1 / sqrt(distance);
            }
            w_near.conservativeResize(w_near.rows(),w_near.cols()+sample_num);
            w_near.rightCols(sample_num) = w.block(0, i * sample_num, w.rows(), sample_num);
        }
        else {
            break;
        }
    }
//    normalize weight
    weight = weight/weight.sum();
    initial_guess = w_near*weight;

    return initial_guess;
}