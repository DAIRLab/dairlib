#include "c3_output.h"

using Eigen::VectorXd;
using std::vector;

namespace dairlib {

C3Output::C3Output(C3Solution c3_sol, C3Intermediates c3_intermediates)
    : c3_solution_(c3_sol), c3_intermediates_(c3_intermediates) {}

lcmt_c3_output C3Output::GenerateLcmObject(double time) const {
  lcmt_c3_output c3_output;
  lcmt_c3_solution c3_solution;
  lcmt_c3_intermediates c3_intermediates;
  c3_output.utime = static_cast<int64_t>(1e6 * time);

  std::cout << c3_solution_.time_vector_.size() << std::endl;
  std::cout << c3_solution_.x_sol_.rows() << std::endl;
  c3_solution.num_points = c3_solution_.time_vector_.size();
  int knot_points = c3_solution.num_points;
  c3_solution.num_state_variables = c3_solution_.x_sol_.rows();
  c3_solution.num_contact_variables = c3_solution_.lambda_sol_.rows();
  c3_solution.num_input_variables = c3_solution_.u_sol_.rows();
  c3_solution.time_vec.reserve(knot_points);
  c3_solution.x_sol = vector<vector<float>>(c3_solution.num_state_variables,
                                            vector<float>(knot_points));
  c3_solution.lambda_sol = vector<vector<float>>(
      c3_solution.num_contact_variables, vector<float>(knot_points));
  c3_solution.u_sol = vector<vector<float>>(c3_solution.num_input_variables,
                                            vector<float>(knot_points));

  c3_solution.time_vec = vector<float>(
      c3_solution_.time_vector_.data(),
      c3_solution_.time_vector_.data() + c3_solution_.time_vector_.size());

  c3_intermediates.num_total_variables = c3_intermediates_.delta_.rows();
  c3_intermediates.num_points = c3_solution.num_points;
  c3_intermediates.time_vec.reserve(c3_intermediates.num_points);
  c3_intermediates.delta_sol = vector<vector<float>>(
      c3_intermediates.num_total_variables, vector<float>(knot_points));
  c3_intermediates.w_sol = vector<vector<float>>(
      c3_intermediates.num_total_variables, vector<float>(knot_points));
  c3_intermediates.time_vec = c3_solution.time_vec;

  for (int i = 0; i < c3_solution.num_state_variables; ++i) {
    // Temporary copy due to underlying data of Eigen::Matrix
    // being column major
    VectorXd temp_row = c3_solution_.x_sol_.row(i);
    memcpy(c3_solution.x_sol[i].data(), temp_row.data(),
           sizeof(float) * knot_points);
  }
  for (int i = 0; i < c3_solution.num_contact_variables; ++i) {
    // Temporary copy due to underlying data of Eigen::Matrix
    // being column major
    VectorXd temp_row = c3_solution_.lambda_sol_.row(i);
    memcpy(c3_solution.lambda_sol[i].data(), temp_row.data(),
           sizeof(float) * knot_points);
  }
  for (int i = 0; i < c3_solution.num_input_variables; ++i) {
    // Temporary copy due to underlying data of Eigen::Matrix
    // being column major
    VectorXd temp_row = c3_solution_.u_sol_.row(i);
    memcpy(c3_solution.u_sol[i].data(), temp_row.data(),
           sizeof(float) * knot_points);
  }
  for (int i = 0; i < c3_intermediates.num_total_variables; ++i) {
    // Temporary copy due to underlying data of Eigen::Matrix
    // being column major
    VectorXd temp_delta_row = c3_intermediates_.delta_.row(i);
    VectorXd temp_w_row = c3_intermediates_.w_.row(i);
    memcpy(c3_intermediates.delta_sol[i].data(), temp_delta_row.data(),
           sizeof(float) * knot_points);
    memcpy(c3_intermediates.w_sol[i].data(), temp_w_row.data(),
           sizeof(float) * knot_points);
  }


  //
  c3_output.c3_solution = c3_solution;
  c3_output.c3_intermediates = lcmt_c3_intermediates();
  std::cout << "constructed c3 output" << std::endl;

  return c3_output;
}

}  // namespace dairlib
