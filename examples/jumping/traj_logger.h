#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <regex>
#include <vector>
#include "drake/solvers/mathematical_program_result.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/goldilocks_models/file_utils.h"

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::RowMajor;
using std::vector;
using dairlib::goldilocks_models::readCSV;
using dairlib::goldilocks_models::writeCSV;

namespace dairlib {
namespace examples {

/*
Generates and writes trajectory to a file in order to visualize in a plot.
*/

void writeTimeTrajToFile(const
                         drake::trajectories::PiecewisePolynomial<double>& traj,
                         std::string filename) {
  std::ofstream* fout = new std::ofstream(filename);
  double timesteps = 500.0;
  for (double t = 0; t < traj.end_time(); t += traj.end_time() / timesteps) {
    (*fout) << t << " ";
    (*fout) << traj.value(t).transpose();
    (*fout) << "\n";
  }
  // std::vector<double> times = traj.get_segment_times();

  // for(size_t i = 0; i < times.size(); ++i){
  //     (*fout) << times[i] << " ";
  //     (*fout) << traj.value(times[i]).transpose();
  //     (*fout) << "\n";
  // }
  fout->flush();
  fout->close();
  delete fout;
}

/*
Writes traj in PiecewisePolynomial format to file in order to be loaded back into a PP
*/
void writePPTrajToFile(const
                       drake::trajectories::PiecewisePolynomial<double>& traj,
                       std::string folderPath, std::string filename) {
  std::ofstream fout;
  fout.open(folderPath + filename);

  int n_segments = traj.get_number_of_segments();
  for (int i = 0; i < n_segments; ++i) {
    fout << traj.getPolynomialMatrix(i);
  }
  // fout << "\n";
  fout.flush();
  fout.close();

  fout.open(folderPath + "times");
  std::vector<double> times = traj.get_segment_times();
  for (size_t i = 0; i < times.size(); ++i) {
    fout << times[i] << " ";
  }
  fout.flush();
  fout.close();
}

void saveAllDecisionVars(drake::solvers::MathematicalProgramResult result,
                         std::string folderPath, std::string filename) {
  std::ofstream fout;
  fout.open(folderPath + filename);

  fout << result.GetSolution();
}

drake::MatrixX<double> loadAllDecisionVars(std::string folderPath,
                                           std::string filename) {
  std::ifstream fin;
  fin.open(folderPath + filename);
  std::string line;
  std::vector<double> decisionVars;

  while (std::getline(fin, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ' ')) {
      decisionVars.push_back(std::stod(cell));
    }
  }
  fin.close();
  return Map<const Matrix<double, Dynamic, Dynamic>>(decisionVars.data(),
                                                     decisionVars.size(), 1);
}

drake::trajectories::PiecewisePolynomial<double> loadTrajToPP(
    std::string folderPath, std::string filename, std::string time_vector,
    int polynomial_order) {
  std::ifstream fin;
  fin.open(folderPath + time_vector);
  std::string line;
  std::vector<double> times;

  while (std::getline(fin, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ' ')) {
      times.push_back(std::stod(cell));
    }
  }
  fin.close();

  fin.open(folderPath + filename);

  std::vector<double> coeffs;
  uint rows = 0;
  while (std::getline(fin, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      coeffs.push_back(std::stod(cell));
    }
    ++rows;
  }
  if (coeffs.size() == 0) {
    throw std::logic_error(
        ("Could not read " + folderPath + " to load CSV.").c_str());
  }

  std::vector<drake::MatrixX<double>> coeff_matrices;
  int n_states = rows / (times.size() - 1);
  int n_coeffs = coeffs.size() / rows; // should always be 4 for a cubic

  // coeff_matrices.push_back(
  //   Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(
  //     coeffs.data() + i * (n_states * n_coeffs), n_states, n_coeffs));

  if (polynomial_order == 1) {
    for (size_t i = 0; i < times.size(); ++i) {
      coeff_matrices.push_back(
          Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(
              coeffs.data() + i * (n_states * n_coeffs), n_coeffs, n_states));
    }
//    return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
//        times,
//        coeff_matrices);
    return drake::trajectories::PiecewisePolynomial<double>::Pchip(
        times,
        coeff_matrices);
  } else if (polynomial_order == 3) {
    for (size_t i = 0; i < times.size(); ++i) {
      coeff_matrices.push_back(
          Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(
              coeffs.data() + i * (n_states * n_coeffs), n_states, n_coeffs));
    }
    return drake::trajectories::PiecewisePolynomial<double>::Cubic(times,
                                                                   coeff_matrices);
  }
  return drake::trajectories::PiecewisePolynomial<double>();
}

}
}