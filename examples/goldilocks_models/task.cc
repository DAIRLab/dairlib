#include "examples/goldilocks_models/task.h"

#include <iostream>

using Eigen::MatrixXi;

namespace dairlib {
namespace goldilocks_models {

TasksGenerator::TasksGenerator(int task_dim, std::vector<string> names,
                               std::vector<int> N_sample_vec)
    : task_dim_(task_dim), names_(names), N_sample_vec_(N_sample_vec) {
  DRAKE_DEMAND(task_dim > 0);
  DRAKE_DEMAND(names.size() == (unsigned)task_dim);
  DRAKE_DEMAND(N_sample_vec.size() == (unsigned)task_dim);
  for (auto n_sample : N_sample_vec) {
    DRAKE_DEMAND(n_sample > 0);
  }

  // Random number generator
  std::random_device randgen;
  random_eng_ = std::default_random_engine(randgen());

  // Non-degenerate task dimension and total sample numbers
  task_dim_nondeg_ = 0;
  N_sample_ = 1;
  for (auto n_sample : N_sample_vec) {
    task_dim_nondeg_ += int(n_sample > 1);
    N_sample_ *= n_sample;
  }

  // Create index map
  for (int i = 0; i < task_dim; i++) {
    name_to_index_map_[names[i]] = i;
  }
}

GridTasksGenerator::GridTasksGenerator(int task_dim, std::vector<string> names,
                                       std::vector<int> N_sample_vec,
                                       std::vector<double> task_0,
                                       std::vector<double> task_delta,
                                       std::vector<bool> is_stochastic)
    : TasksGenerator(task_dim, names, N_sample_vec),
      task_0_(task_0),
      task_delta_(task_delta),
      is_stochastic_(is_stochastic) {
  DRAKE_DEMAND(task_0.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_delta.size() == (unsigned)task_dim);

  // Construct forward and backward index map
  int i_layer = 0;
  int sample_idx = 0;
  vector<int> index_tuple(task_dim, 0);
  RunThroughIndex(N_sample_vec, i_layer, index_tuple, &sample_idx,
                  &forward_task_idx_map_, &inverse_task_idx_map_);

  // Tasks setup
  for (int i = 0; i < task_dim; i++) {
    // Distribution
    distribution_.emplace_back(-task_delta[i] / 2, task_delta[i] / 2);
    // Task grid
    vector<double> sub_task_grid;
    for (int j = 0 - N_sample_vec[i] / 2;
         j < N_sample_vec[i] - N_sample_vec[i] / 2; j++)
      sub_task_grid.push_back(j * task_delta[i]);
    task_grid_.push_back(sub_task_grid);
    // Min
    task_min_range_.push_back((is_stochastic[i] && (N_sample_vec[i] > 0))
                                  ? task_0[i] + sub_task_grid.front() -
                                        task_delta[i] * 0.5
                                  : task_0[i] + sub_task_grid.front());
    // Max
    task_max_range_.push_back((is_stochastic[i] && (N_sample_vec[i] > 0))
                                  ? task_0[i] + sub_task_grid.back() +
                                        task_delta[i] * 0.5
                                  : task_0[i] + sub_task_grid.back());
  }
}

void GridTasksGenerator::PrintInfo() const {
  for (int i = 0; i < dim(); i++) {
    cout << names()[i] << ": \n";
    cout << "  # of samples = " << sample_numbers()[i] << endl;
    cout << "  center = " << task_0_[i] << endl;
    cout << "  spacing = " << task_delta_[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> GridTasksGenerator::NewNominalTask(int sample_idx) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
  }
  return ret;
}

vector<double> GridTasksGenerator::NewTask(int sample_idx) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
    if (is_stochastic_[i]) {
      if (N_sample_vec_[i] > 0) {
        ret[i] += distribution_[i](random_eng_);
      }
    }
  }
  return ret;
}

Eigen::MatrixXi GridTasksGenerator::CreateSampleIdxAdjacentMatrix(
    vector<int> n_node_vec) const {
  // Setup
  int task_dim = this->dim_nondeg();
  int N_sample = this->total_sample_number();

  if (n_node_vec.empty()) {
    n_node_vec = std::vector<int>(N_sample, 1);
  }

  // cout << "Constructing adjacent index list...\n";
  MatrixXi adjacent_sample_indices =
      -1 * MatrixXi::Ones(N_sample, 2 * task_dim);
  MatrixXi delta_idx = MatrixXi::Identity(3, 3);
  for (int sample_idx = 0; sample_idx < N_sample; sample_idx++) {
    for (int i = 0; i < task_dim; i++) {
      vector<int> new_index_tuple = forward_task_idx_map_.at(sample_idx);
      new_index_tuple[i] += 1;

      // The new index tuple has to be valid
      if (new_index_tuple[i] < this->sample_numbers()[i]) {
        int adjacent_sample_idx = inverse_task_idx_map_.at(new_index_tuple);
        // Number of nodes should be the same so that we can set initial guess
        if (n_node_vec[sample_idx] == n_node_vec[adjacent_sample_idx]) {
          // Add to adjacent_sample_idx (both directions)
          for (int l = 0; l < 2 * task_dim; l++) {
            if (adjacent_sample_indices(sample_idx, l) < 0) {
              adjacent_sample_indices(sample_idx, l) = adjacent_sample_idx;
              break;
            }
          }
          for (int l = 0; l < 2 * task_dim; l++) {
            if (adjacent_sample_indices(adjacent_sample_idx, l) < 0) {
              adjacent_sample_indices(adjacent_sample_idx, l) = sample_idx;
              break;
            }
          }
        }
      }
    }
  }
  return adjacent_sample_indices;
}

void GridTasksGenerator::RunThroughIndex(
    const std::vector<int>& N_sample, int i_layer, vector<int> index_tuple,
    int* sample_idx, std::map<int, std::vector<int>>* forward_task_idx_map,
    std::map<std::vector<int>, int>* inverse_task_idx_map) {
  if ((unsigned)i_layer == N_sample.size()) {
    (*forward_task_idx_map)[(*sample_idx)] = index_tuple;
    (*inverse_task_idx_map)[index_tuple] = (*sample_idx);
    (*sample_idx) += 1;
  } else {
    while (index_tuple[i_layer] < N_sample[i_layer]) {
      RunThroughIndex(N_sample, i_layer + 1, index_tuple, sample_idx,
                      forward_task_idx_map, inverse_task_idx_map);
      index_tuple[i_layer] += 1;
    }
  }
}

// Tasks are randomly generated from the whole optimization space
UniformTasksGenerator::UniformTasksGenerator(
    int task_dim, std::vector<string> names, std::vector<int> N_sample_vec,
    const std::vector<double>& task_min, const std::vector<double>& task_max)
    : TasksGenerator(task_dim, names, N_sample_vec) {
  DRAKE_DEMAND(task_min.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_max.size() == (unsigned)task_dim);

  task_min_range_ = task_min;
  task_max_range_ = task_max;

  // Tasks distribution
  for (int i = 0; i < task_dim; i++) {
    // Distribution
    distribution_.emplace_back(task_min_range_[i], task_max_range_[i]);
  }
}

void UniformTasksGenerator::PrintInfo() const {
  for (int i = 0; i < dim(); i++) {
    cout << names()[i] << ": \n";
    cout << "  # of samples = " << sample_numbers()[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}
vector<double> UniformTasksGenerator::NewTask(int sample_idx) {
  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = distribution_[i](random_eng_);
  }
  return ret;
}

}  // namespace goldilocks_models
}  // namespace dairlib
