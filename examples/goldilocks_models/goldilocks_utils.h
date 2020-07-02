#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/goldilocks_models/file_utils.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace dairlib {
namespace goldilocks_models  {

class InnerLoopSetting {
 public:
  InnerLoopSetting() {};

  int n_node;
  double Q_double;
  double R_double;
  double eps_reg;
  double all_cost_scale;
  bool is_add_tau_in_cost;
  bool is_zero_touchdown_impact;

  int max_iter;
  double major_optimality_tol;
  double major_feasibility_tol;
  bool snopt_scaling;

  string directory;
  string prefix;
  string init_file;
};

class RomData {
 public:
  RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot)
      : RomData(n_y, n_tau, n_feature_y, n_feature_yddot,
                MatrixXd::Zero(n_y, n_tau), VectorXd::Zero(n_y * n_feature_y),
                VectorXd::Zero(n_y * n_feature_yddot)){};

  RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot,
          const MatrixXd& B_tau, const VectorXd& theta_y,
          const VectorXd& theta_yddot)
      : n_y_(n_y),
        n_yddot_(n_y),
        n_tau_(n_tau),
        n_feature_y_(n_feature_y),
        n_feature_yddot_(n_feature_yddot),
        B_tau_(B_tau),
        theta_y_(theta_y),
        theta_yddot_(theta_yddot) {
    CheckDataConsistency();
  };

  void CheckDataConsistency() {
    DRAKE_DEMAND(B_tau_.rows() == n_y_);
    DRAKE_DEMAND(B_tau_.cols() == n_tau_);
    DRAKE_DEMAND(theta_y_.size() == n_y_ * n_feature_y_);
    DRAKE_DEMAND(theta_yddot_.size() == n_y_ * n_feature_yddot_);
  };

  // Getters
  int n_y() const {return n_y_;};
  int n_yddot() const {return n_y_;};
  int n_tau() const {return n_tau_;};
  int n_feature_y() const {return n_feature_y_;};
  int n_feature_yddot() const {return n_feature_yddot_;};
  const MatrixXd& B() const {return B_tau_;};
  const VectorXd& theta_y() const {return theta_y_;};
  const VectorXd& theta_yddot() const { return theta_yddot_; };
  VectorXd theta() const {
    VectorXd ret(theta_y_.size() + theta_yddot_.size());
    ret << theta_y_, theta_yddot_;
    return ret;
  };
  int n_theta_y() const {return theta_y_.size();};
  int n_theta_yddot() const {return theta_yddot_.size();};
  int n_theta() const {return theta_y_.size() + theta_yddot_.size();};

  // Setters
  void set_n_y(int n_y) { n_y_ = n_y; };
  void set_n_yddot(int n_yddot) { n_yddot_ = n_yddot; };
  void set_n_tau(int n_tau) { n_tau_ = n_tau; };
  void set_n_feature_y(int n_feature_y) { n_feature_y_ = n_feature_y; };
  void set_n_feature_yddot(int n_feature_yddot) {
    n_feature_yddot_ = n_feature_yddot;
  };
  void SetB(const MatrixXd& B_tau) {
    if ((B_tau_.rows() == B_tau.rows()) && (B_tau_.cols() == B_tau.cols())) {
      B_tau_ = B_tau;
    } else {
      B_tau_.resizeLike(B_tau);
      B_tau_ = B_tau;
    }
  };
  void SetThetaY(const VectorXd& theta_y) {
    if (theta_y_.size() == theta_y.size()) {
      theta_y_ = theta_y;
    } else {
      theta_y_.resizeLike(theta_y);
      theta_y_ = theta_y;
    }
  };
  void SetThetaYddot(const VectorXd& theta_yddot) {
    if (theta_yddot_.size() == theta_yddot.size() ) {
      theta_yddot_ = theta_yddot;
    } else {
      theta_yddot_.resizeLike(theta_yddot);
      theta_yddot_ = theta_yddot;
    }
  };
  void SetTheta(const VectorXd& theta) {
    DRAKE_DEMAND(theta.size() == theta_y_.size() + theta_yddot_.size());
    theta_y_ = theta.head(theta_y_.size());
    theta_yddot_ = theta.tail(theta_yddot_.size());
  };

 private:
  int n_y_;
  int n_yddot_;
  int n_tau_;
  int n_feature_y_;
  int n_feature_yddot_;

  MatrixXd B_tau_;
  VectorXd theta_y_;
  VectorXd theta_yddot_;
};

// SubQpData stores all the data about the QPs in the SQP algorithm
class SubQpData {
 public:
  // Constructor
  SubQpData(int N_sample);

  // Vectors/Matrices for the outer loop
  vector<std::shared_ptr<VectorXd>> w_sol_vec;
  vector<std::shared_ptr<MatrixXd>> H_vec;
  vector<std::shared_ptr<VectorXd>> b_vec;
  vector<std::shared_ptr<VectorXd>> c_vec;
  vector<std::shared_ptr<MatrixXd>> A_vec;
  vector<std::shared_ptr<VectorXd>> lb_vec;
  vector<std::shared_ptr<VectorXd>> ub_vec;
  vector<std::shared_ptr<VectorXd>> y_vec;
  vector<std::shared_ptr<MatrixXd>> B_vec;
  vector<std::shared_ptr<int>> is_success_vec;

  // Vectors/Matrices for the outer loop (when cost descent is successful)
  vector<std::shared_ptr<MatrixXd>> A_active_vec;
  vector<std::shared_ptr<MatrixXd>> B_active_vec;
  vector<std::shared_ptr<int>> nw_vec;  // size of traj opt dec var
  vector<std::shared_ptr<int>> nl_vec;  // # of active constraints
  vector<std::shared_ptr<MatrixXd>> P_vec;  // w = P_i * theta + q_i
  vector<std::shared_ptr<VectorXd>> q_vec;  // w = P_i * theta + q_i
};

// Create cubic splines from s and sdot
PiecewisePolynomial<double> createCubicSplineGivenSAndSdot(
  const vector<VectorXd> & h_vec,
  const vector<VectorXd> & s_vec, const vector<VectorXd> & ds_vec);

// Store splines in csv file for plotting
// The first row is time, and the rest rows are s
void storeSplineOfS(const vector<VectorXd> & h_vec,
                    const PiecewisePolynomial<double> & s_spline,
                    const string & directory, const string & prefix);

// Check whether your cubic spline implemented in dynamics constriant is correct
void checkSplineOfS(const vector<VectorXd> & h_vec,
                    const vector<VectorXd> & dds_vec,
                    const PiecewisePolynomial<double> & s_spline);

void storeTau(const vector<VectorXd> & h_vec,
              const vector<VectorXd> & tau_vec,
              const string & directory,
              const string & prefix);

VectorXd createPrimeNumbers(int num_prime);

bool file_exist (const std::string & name);
bool folder_exist (const std::string & pathname_string);
// return false when the user want to stop
bool CreateFolderIfNotExist(const string& dir, bool ask_for_permission = true);

vector<std::string> ParseCsvToStringVec(const std::string& file_name);
void SaveStringVecToCsv(vector<std::string> strings,
                        const std::string& file_name);

}  // namespace goldilocks_models
} // dairlib

