#include <iostream>
#include <string>
#include "math.h"
#include <Eigen/Dense>

using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::VectorXcd;


MatrixXd solveInvATimesB(const MatrixXd & A, const MatrixXd & B) {
  MatrixXd X = (A.transpose() * A).ldlt().solve(A.transpose() * B);
  MatrixXd abs_resid = (A * X - B).cwiseAbs();
  VectorXd left_one = VectorXd::Ones(abs_resid.rows());
  VectorXd right_one = VectorXd::Ones(abs_resid.cols());
  cout << "sum-abs-residual: " << left_one.transpose()*abs_resid*right_one <<
       endl;
  return X;
}

int main() {

  // Reference: https://eigen.tuxfamily.org/dox/group__LeastSquares.html
  MatrixXd A = MatrixXd::Random(3, 2);
  VectorXd b = VectorXd::Random(3);
  cout << "b = " << b << endl;
  cout << "The solution using normal equations is:\n"
     << (A.transpose() * A).ldlt().solve(A.transpose() * b) << endl;
  cout << "Using the function: \n" << solveInvATimesB(A,b)<< endl;

  MatrixXd B = MatrixXd::Random(3, 2);
  B << b,b;
  cout << "B = " << B << endl;
  cout << "The solution using normal equations is:\n"
     << (A.transpose() * A).ldlt().solve(A.transpose() * B) << endl;
  cout << "Using the function: \n" << solveInvATimesB(A,B)<< endl;

  // Directly use inverse()
  // http://eigen.tuxfamily.org/dox-devel/classEigen_1_1MatrixBase.html#title61
  A = MatrixXd::Random(3, 3);
  B = MatrixXd::Random(3, 2);
  cout << "The solution using normal equations is:\n"
     << (A.transpose() * A).ldlt().solve(A.transpose() * B) << endl;
  cout << "Using the function: \n" << solveInvATimesB(A,B)<< endl;
  cout << "Solving directly using inv(): \n" << A.inverse()*B << endl;

  // Other methods that you can try
  // x = A.lu().solve(b);
  // x = A.fullPivLu().solve(b);
  // x = A.colPivHouseholderQr().solve(b);
  // x = A.jacobiSVD(ComputeThinU|ComputeThinV).solve(b);




  /*// Testing eigenvalues
  MatrixXd ones = MatrixXd::Random(3,3);
  ones = ones + ones.transpose();
  cout << "ones = \n" << ones << "\n\n";
  VectorXcd eivals = ones.eigenvalues();
  cout << "The eigenvalues of the 3x3 matrix of ones are:\n" << endl << eivals << endl;
  cout<< "real part = \n"<<eivals.real()<<endl;*/

  /*// Testing abs
  cout << ones.cwiseAbs() << endl;*/

  /*// Testing sigular value
  Eigen::BDCSVD<MatrixXd> svd(A);
  int n_sv = svd.singularValues().size();
  cout << "smallest singular value is " << svd.singularValues()(n_sv - 1) << endl;
  cout << "singular values are \n" << svd.singularValues() << endl;*/

  // Testing if I can change the size of MatrixXd
  MatrixXd A_temp = MatrixXd::Zero(1,1);
  cout << "A_temp = \n" << A_temp << endl;
  A_temp = MatrixXd::Zero(2,1);
  cout << "A_temp = \n" << A_temp << endl;


  return 0;
}
