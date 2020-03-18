//
// Created by jianshu on 3/16/20.
//

#include <iostream>
#include <Eigen/Dense>
using Eigen::VectorXd;
using Eigen::MatrixXd;

class set_initial_guess{
public:
/*
 * define variables used to restore the results of previous trajectory optimizations
 * theta:the ith column contains the theta for ith iteration
 * gamma:the ith block(from (i-1)*N+1 column to i*N column) contains the gamma for ith iteration
 * w:from ith block(from (i-1)*N+1 column to i*N column) contains the w for ith iterartion
 * N:number of gamma samples in each iteration
 */
    MatrixXd theta;
    MatrixXd gamma;
    MatrixXd w;
    int sample_num;
    set_initial_guess(VectorXd& initial_theta, int samples_number);
    void add_theta(VectorXd& result_theta);
    void add_gamma(VectorXd& sample_gamma);
    void add_w(VectorXd& result_w);
    VectorXd predict(VectorXd& new_theta, VectorXd& new_gamma);
private:
/* define some parameters used in interpolation
 * theta_range :decide the range of theta to use in interpolation
 * theta_sclae,gamma_scale :used to scale the theta and gamma in interpolation
 */
    double theta_range = 0.2;
    double theta_scale = 1;
    double gamma_scale = 1;
};
