#pragma once
#include <Eigen/Dense>

int SolveLCP(Eigen::MatrixXd* M, Eigen::VectorXd* q, Eigen::VectorXd* z);
