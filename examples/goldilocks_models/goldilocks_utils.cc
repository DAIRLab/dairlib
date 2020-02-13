#include "examples/goldilocks_models/goldilocks_utils.h"

namespace dairlib {
namespace goldilocks_models  {

// Create time knots for creating cubic splines
vector<double> createTimeKnotsGivenTimesteps(const vector<VectorXd> & h_vec) {
  vector<double> T_breakpoint;
  double time = 0;
  T_breakpoint.push_back(time);
  for (unsigned int i = 0; i < h_vec.size() ; i++) {
    time += h_vec[i](0);
    T_breakpoint.push_back(time);
  }
  return T_breakpoint;
}


PiecewisePolynomial<double> createCubicSplineGivenSAndSdot(
  const vector<VectorXd> & h_vec,
  const vector<VectorXd> & s_vec,
  const vector<VectorXd> & ds_vec) {
  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create traj value and its derivatives (convert VectorXd to MatrixXd)
  vector<MatrixXd> s(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  vector<MatrixXd> s_dot(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  for (unsigned int i = 0; i < s_vec.size() ; i++) {
    s[i] = s_vec[i];
    s_dot[i] = ds_vec[i];
  }

  // Construct splines
  return PiecewisePolynomial<double>::Cubic(T_breakpoint, s, s_dot);
}


void storeSplineOfS(const vector<VectorXd> & h_vec,
                    const PiecewisePolynomial<double> & s_spline,
                    const string & directory,
                    const string & prefix) {
  // parameters
  int n_sample_each_seg = 3;

  // setup
  int n_s = s_spline.value(0).rows();

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create the matrix for csv file
  // The first row is time, and the rest rows are s
  MatrixXd t_and_s(1 + n_s, 1 + (n_sample_each_seg - 1)*h_vec.size());
  MatrixXd t_and_ds(1 + n_s, 1 + (n_sample_each_seg - 1)*h_vec.size());
  MatrixXd t_and_dds(1 + n_s, 1 + (n_sample_each_seg - 1)*h_vec.size());
  t_and_s(0, 0) = 0;
  t_and_ds(0, 0) = 0;
  t_and_dds(0, 0) = 0;
  t_and_s.block(1, 0, n_s, 1) = s_spline.value(0);
  t_and_ds.block(1, 0, n_s, 1) = s_spline.derivative(1).value(0);
  t_and_dds.block(1, 0, n_s, 1) = s_spline.derivative(2).value(0);
  for (unsigned int i = 0; i < h_vec.size() ; i++) {
    for (int j = 1; j < n_sample_each_seg; j++) {
      double time = T_breakpoint[i] + j * h_vec[i](0) / (n_sample_each_seg - 1);
      t_and_s(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_ds(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_dds(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_s.block(1, j + i * (n_sample_each_seg - 1), n_s, 1) =
        s_spline.value(time);
      t_and_ds.block(1, j + i * (n_sample_each_seg - 1), n_s, 1) =
        s_spline.derivative(1).value(time);
      t_and_dds.block(1, j + i * (n_sample_each_seg - 1), n_s, 1) =
        s_spline.derivative(2).value(time);
    }
  }

  // Store into csv file
  writeCSV(directory + prefix + string("t_and_s.csv"), t_and_s);
  writeCSV(directory + prefix + string("t_and_ds.csv"), t_and_ds);
  writeCSV(directory + prefix + string("t_and_dds.csv"), t_and_dds);
}


void checkSplineOfS(const vector<VectorXd> & h_vec,
                    const vector<VectorXd> & dds_vec,
                    const PiecewisePolynomial<double> & s_spline) {
  // parameters
  double tol = 1e-4;

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Compare
  for (unsigned int i = 0; i < T_breakpoint.size() ; i++) {
    VectorXd dds_by_drake = s_spline.derivative(2).value(T_breakpoint[i]);
    VectorXd dds_by_hand = dds_vec[i];
    DRAKE_DEMAND((dds_by_drake - dds_by_hand).norm() < tol);
  }
}


void storeTau(const vector<VectorXd> & h_vec,
              const vector<VectorXd> & tau_vec,
              const string & directory,
              const string & prefix) {
  // setup
  int n_tau = tau_vec[0].rows();

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create the matrix for csv file
  // The first row is time, and the rest rows are tau
  MatrixXd t_and_tau(1 + n_tau, tau_vec.size());
  for (unsigned int i = 0; i < T_breakpoint.size() ; i++) {
    t_and_tau(0, i) = T_breakpoint[i];
    t_and_tau.block(1, i, n_tau, 1) = tau_vec[i];
  }

  // Store into csv file
  writeCSV(directory + prefix + string("t_and_tau.csv"), t_and_tau);
}


VectorXd createPrimeNumbers(int num_prime) {
  DRAKE_DEMAND(num_prime <= 25);

  VectorXd prime_until_100(25);
  prime_until_100 << 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53,
                  59, 61, 67, 71, 73, 79, 83, 89, 97;
  return prime_until_100.head(num_prime);
}



}  // namespace goldilocks_models
} // dairlib

