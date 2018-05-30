#include <memory>
#include <chrono>
#include <random>
#include <gflags/gflags.h>

#include <Eigen/Sparse>

#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

#include "sgd_iter.h"
#include "systems/goldilocks_models/file_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::goldilocks_models::writeCSV;
using drake::goldilocks_models::readCSV;
using drake::solvers::VectorXDecisionVariable;

namespace drake{
namespace goldilocks_models {

void runSGD() {
  std::random_device randgen;
  std::default_random_engine e1(randgen());
  std::uniform_real_distribution<> dist(0, 1);


  int n_batch = 5;

  // int n_weights = 43;
  int n_weights =  10;
  MatrixXd theta_0 = MatrixXd::Zero(2,n_weights);
  theta_0(0,0) = -0.1;
  theta_0(0,3) = 1.0;
  theta_0(1,0) = 0;
  theta_0(1,1) = 1;
  writeCSV("data/0_theta.csv", theta_0);

  double length = 0.3;
  double duration = 1;
  int snopt_iter = 200;
  string directory = "data/";
  string init_z = "z_save.csv";
  string weights = "0_theta.csv";
  string output_prefix = "0_0_";
  sgdIter(length, duration, snopt_iter, directory, init_z, weights, output_prefix);

  for (int iter = 1; iter <= 50; iter++) {
    int input_batch = iter == 1 ? 1 : n_batch;

    std::vector<MatrixXd> A_vec;
    std::vector<MatrixXd> B_vec;
    std::vector<MatrixXd> H_vec;
    std::vector<MatrixXd> A_active_vec;
    std::vector<MatrixXd> B_active_vec;
    std::vector<MatrixXd> lb_vec;
    std::vector<MatrixXd> ub_vec;
    std::vector<MatrixXd> y_vec;
    std::vector<MatrixXd> w_vec;
    std::vector<MatrixXd> z_vec;
    std::vector<MatrixXd> theta_vec;
    std::vector<double> nl_vec;
    std::vector<double> nz_vec;

    int nz=0,nt=0,nl=0;

    for (int batch = 0; batch < input_batch; batch++) {
      string batch_prefix = std::to_string(iter-1) + "_" + std::to_string(batch) + "_";
      string iter_prefix = std::to_string(iter-1) + "_";

      A_vec.push_back(readCSV(directory + batch_prefix + "A.csv"));
      B_vec.push_back(readCSV(directory + batch_prefix + "B.csv"));
      H_vec.push_back(readCSV(directory + batch_prefix + "H.csv"));
      lb_vec.push_back(readCSV(directory + batch_prefix + "lb.csv"));
      ub_vec.push_back(readCSV(directory + batch_prefix + "ub.csv"));
      y_vec.push_back(readCSV(directory + batch_prefix + "y.csv"));
      w_vec.push_back(readCSV(directory + batch_prefix + "w.csv"));
      z_vec.push_back(readCSV(directory + batch_prefix + "z.csv"));
      theta_vec.push_back(readCSV(directory + iter_prefix + "theta.csv"));

      DRAKE_ASSERT(w_vec[batch].cols() == 1);
      DRAKE_ASSERT(lb_vec[batch].cols() == 1);
      DRAKE_ASSERT(ub_vec[batch].cols() == 1);
      DRAKE_ASSERT(y_vec[batch].cols() == 1);
      DRAKE_ASSERT(w_vec[batch].cols() == 1);
      DRAKE_ASSERT(z_vec[batch].cols() == 1);


      int n_active = 0;
      double tol = 1e-4;
      for (int i = 0; i < y_vec[batch].rows(); i++) {
        if (y_vec[batch](i) >= ub_vec[batch](i) - tol || y_vec[batch](i) <= lb_vec[batch](i) + tol)
          n_active++;
      }

      int nz_i = A_vec[batch].cols();
      int nt_i = B_vec[batch].cols();

      MatrixXd A_active(n_active, nz_i);
      MatrixXd B_active(n_active, nt_i);
      MatrixXd AB_active(n_active, nz_i + nt_i);

      int nl_i = 0;
      for (int i = 0; i < y_vec[batch].rows(); i++) {
        if (y_vec[batch](i) >= ub_vec[batch](i) - tol || y_vec[batch](i) <= lb_vec[batch](i) + tol) {
          A_active.row(nl_i) = A_vec[batch].row(i);
          B_active.row(nl_i) = B_vec[batch].row(i);
          AB_active.row(nl_i) << A_vec[batch].row(i), B_vec[batch].row(i);
          nl_i++;
        }
      }

      A_active_vec.push_back(A_active);
      B_active_vec.push_back(B_active);
      nl_vec.push_back(nl_i);
      nz_vec.push_back(nz_i);

      nl += nl_i;
      nz += nz_i;
      if (batch == 0) {
        nt = nt_i;
      } else {
        DRAKE_ASSERT(nt == nt_i);
        DRAKE_ASSERT((theta_vec[0] - theta_vec[batch]).norm() == 0);
      }
    }

    //Join matricies
    // MatrixXd AB_active = MatrixXd::Zero(nl,nz+nt);
    // MatrixXd H_ext = MatrixXd::Zero(nz + nt, nz + nt);
    VectorXd w_ext = VectorXd::Zero(nz+nt,1);
    int nl_start = 0;
    int nz_start = 0;
    for (int batch = 0; batch < input_batch; batch++) {
      // AB_active.block(nl_start, nz_start, nl_vec[batch], nz_vec[batch]) = A_active_vec[batch];
      // AB_active.block(nl_start, nz, nl_vec[batch], nt) = B_active_vec[batch];

      // H_ext.block(nz_start,nz_start,nz_vec[batch],nz_vec[batch]) = H_vec[batch];
      w_ext.segment(nz_start,nz_vec[batch]) = w_vec[batch].col(0);

      nl_start += nl_vec[batch];
      nz_start += nz_vec[batch];
    }
    // H_ext.block(nz,nz,nt,nt) = 1e-2*MatrixXd::Identity(nt,nt);


    // Eigen::BDCSVD<MatrixXd> svd(AB_active,  Eigen::ComputeFullV);

    // MatrixXd N = svd.matrixV().rightCols(AB_active.cols() - svd.rank());

    // auto gradient = N*N.transpose()*w_ext;

    // double scale_num= gradient.dot(gradient);
    // double scale_den = gradient.dot(Ei_ext*gradient);

    // auto dtheta = -0.02*gradient.tail(nt)*scale_num/scale_den;


    nl_start = 0;
    nz_start = 0;
    std::vector<Eigen::Triplet<double>> tripletList;
    std::vector<Eigen::Triplet<double>> tripletList_H;
    for (int batch = 0; batch < input_batch; batch++) {
      for (int i = 0; i < nz_vec[batch]; i++) {
        for (int j = 0; j < nz_vec[batch]; j++) {
          tripletList.push_back(Eigen::Triplet<double>(nz_start + i, nz_start + j, H_vec[batch](i,j)));
          tripletList_H.push_back(Eigen::Triplet<double>(nz_start + i, nz_start + j, H_vec[batch](i,j)));
        }
      }
      for (int i = 0; i < nl_vec[batch]; i++) {
        for (int j = 0; j < nz_vec[batch]; j++) {
          int i_ind = nz + nt + nl_start + i;
          int j_ind = nz_start + j;
          tripletList.push_back(Eigen::Triplet<double>(i_ind, j_ind, A_active_vec[batch](i,j)));
          tripletList.push_back(Eigen::Triplet<double>(j_ind, i_ind, A_active_vec[batch](i,j)));
        }
        for (int j = 0; j < nt; j++) {
          int i_ind = nz + nt + nl_start + i;
          int j_ind = nz + j;
          tripletList.push_back(Eigen::Triplet<double>(i_ind, j_ind, B_active_vec[batch](i,j)));
          tripletList.push_back(Eigen::Triplet<double>(j_ind, i_ind, B_active_vec[batch](i,j)));
        }
      }
      nl_start += nl_vec[batch];
      nz_start += nz_vec[batch];
    }

    VectorXd b(nz + nt + nl);
    b << -w_ext, VectorXd::Zero(nl);

    Eigen::SparseMatrix<double> M(nz + nt + nl, nz + nt + nl);
    M.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::SparseMatrix<double> H_ext(nz + nt, nz + nt);
    H_ext.setFromTriplets(tripletList_H.begin(), tripletList_H.end());
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr(M);
    auto gradient = qr.solve(b);

    // Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    // // fill A and b;
    // // Compute the ordering permutation vector from the structural pattern of A
    // solver.analyzePattern(M); 
    // // Compute the numerical factorization
    // solver.factorize(M);
    // std::cout << solver.info() << std::endl;
    // //Use the factors to solve the linear system
    // auto gradient = solver.solve(b);

    // Eigen::BiCGSTAB<Eigen::SparseMatrix<double> > solver;
    // solver.compute(M);
    // auto gradient = solver.solve(b);
    // std::cout << "#iterations:     " << solver.iterations() << std::endl;
    // std::cout << "estimated error: " << solver.error()      << std::endl;

    // std::cout << M*gradient - b << std::endl;


    // std::cout << gradient << std::endl;


    // //M = [H_ext A'; A 0]
    // MatrixXd M(nz + nt + nl, nz + nt + nl);
    // M.block(0, 0, nz + nt, nz + nt) = H_ext;
    // M.block(0, nz + nt, nz + nt, nl) = AB_active.transpose();
    // M.block(nz + nt, 0, nl, nz + nt) = AB_active;
    // M.block(nz + nt, nz + nt, nl, nl) = MatrixXd::Zero(nl,nl);
    // VectorXd b(nz + nt + nl);
    // b << -w_ext, VectorXd::Zero(nl);
    // auto gradient = M.colPivHouseholderQr().solve(b);

    auto zt = gradient.head(nz+nt);

    auto resid = M*gradient - b;

    std::cout << "residual-norm: "<< resid.tail(nl).norm() << std::endl;
    std::cout << "descent: "<< gradient.head(nz+nt).dot(w_ext) << std::endl;

    double scale = 1/sqrt(zt.dot(H_ext*zt));

    std::cout << "scale: "<< scale << std::endl;

    auto dtheta = -.01*scale*gradient.segment(nz, nt);


    std::cout << "found dtheta"<< std::endl;

    std::cout << std::endl<< "dtheta norm: " << dtheta.norm() << std::endl;
    std::cout << "***********Next iteration*************" << std::endl;

    // std::cout << "scale predict: " << scale_num/scale_den << std::endl;

    //reshape dtheta
    MatrixXd theta_mat(theta_vec[0].rows(), theta_vec[0].cols());
    for (int i = 0; i < theta_vec[0].rows(); i++) {
      theta_mat.row(i) = theta_vec[0].row(i) +
                         dtheta.segment(i*theta_vec[0].cols(),theta_vec[0].cols()).transpose();
    }
    if (iter == 1)
      writeCSV("data/" + std::to_string(iter) + "_theta.csv", theta_vec[0]);
    else
      writeCSV("data/" + std::to_string(iter) + "_theta.csv", theta_mat);

    // init_z = std::to_string(iter-1) + "_z.csv";
    init_z = "z_save.csv";
    weights = std::to_string(iter) +  "_theta.csv";
    output_prefix = std::to_string(iter) +  "_";

    for(int batch = 0; batch < n_batch; batch++) {
    //randomize distance on [0.3,0.5]
      // length = 0.2 + 0.3*dist(e1);
      length = 0.15 + 0.05*batch + 0.0*dist(e1);
      // duration =  length/0.5; //maintain constaint speed of 0.5 m/s
      duration = 1;

      int length_file_index = (int) ((0.5 - length) * 10);
      if (iter == 1) 
        init_z = "init_length_" + std::to_string(length_file_index) + "_speed_0_z.csv";
      else
        init_z = std::to_string(iter-1) + "_" + std::to_string(batch) + "_z.csv";

      std::cout << std::endl << "Iter-Batch: " << iter << "-" << batch << std::endl;
      std::cout << "New length: " << length << std::endl;

      string batch_prefix = output_prefix + std::to_string(batch) + "_";

      sgdIter(length, duration, snopt_iter, directory, init_z, weights, batch_prefix);
    }
  }
}
}
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  drake::goldilocks_models::runSGD();
}
