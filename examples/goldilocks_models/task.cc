#include "examples/goldilocks_models/task.h"

#include <iostream>

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

VectorXd TasksGenerator::GetTaskScale() const {
  int task_dimension = task_dim_;
  VectorXd task_scale = VectorXd::Zero(task_dimension);
  // if not fixed task, we need to scale the task
  int dim = 0;
  double min;
  double max;
  for (dim = 0; dim < task_dimension; dim++) {
    min = task_min(names_[dim]);
    max = task_max(names_[dim]);
    if (!(min == max)) {
      // coefficient is different for different dimensions
      if (names_[dim] == "turning_rate") {
        task_scale[dim] = 1.3 / (max - min);
      } else {
        task_scale[dim] = 1 / (max - min);
      }
    }
  }
  return task_scale;
}

GridTasksGenerator::GridTasksGenerator(int task_dim, std::vector<string> names,
                                       std::vector<int> N_sample_vec,
                                       std::vector<double> task_0,
                                       std::vector<double> task_delta,
                                       bool is_stochastic)
    : TasksGenerator(task_dim, names, N_sample_vec),
      task_0_(task_0),
      task_delta_(task_delta),
      is_stochastic_(is_stochastic) {
  DRAKE_DEMAND(task_0.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_delta.size() == (unsigned)task_dim);

  // we don't implement the feature of extending task space for grid method
  currently_extend_task_space_ = false;
  iter_start_optimization_ = 1;

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
    task_min_range_.push_back((is_stochastic && (N_sample_vec[i] > 0))
                                  ? task_0[i] + sub_task_grid.front() -
                                        task_delta[i] * 0.5
                                  : task_0[i] + sub_task_grid.front());
    // Max
    task_max_range_.push_back((is_stochastic && (N_sample_vec[i] > 0))
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

vector<double> GridTasksGenerator::NewTask(string dir, int iter,int sample_idx) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
    if (is_stochastic_) {
      if (N_sample_vec_[i] > 0) {
        ret[i] += distribution_[i](random_eng_);
      }
    }
  }
  return ret;
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
    const std::vector<double>& task_min, const std::vector<double>& task_max,
    int iter_start_optimization)
    : TasksGenerator(task_dim, names, N_sample_vec) {
  DRAKE_DEMAND(task_min.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_max.size() == (unsigned)task_dim);
  DRAKE_DEMAND(iter_start_optimization>0);
  // initially we extend the task space
  currently_extend_task_space_ = true;
  iter_start_optimization_ = iter_start_optimization;;

  task_min_range_ = task_min;
  task_max_range_ = task_max;

  // Tasks distribution
  for (int i = 0; i < task_dim; i++) {
    // Distribution
    distribution_.emplace_back(task_min_range_[i], task_max_range_[i]);
  }

  //set the flag of extend the task space as true
  currently_extend_task_space_ = true;
}

void UniformTasksGenerator::PrintInfo() const {
  for (int i = 0; i < dim(); i++) {
    cout << names()[i] << ": \n";
    cout << "  # of samples = " << sample_numbers()[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> UniformTasksGenerator::NewTask(string dir,int iter,int sample_idx) {
  vector<double> ret(task_dim_, 0);
  //the task space is gradually pushed until reach the final optimization range
  if(iter<iter_start_optimization_){
    //extend the task space
    currently_extend_task_space_ = true;
    //decide the range of optimization by the number of iteration
    double central;
    double interval;
    double new_task_max_range;
    double new_task_min_range;
    for (int i = 0; i < task_dim_; i++) {
      central = (task_max_range_[i]+task_min_range_[i])/2;
      interval = (task_max_range_[i]-task_min_range_[i])/2;
      new_task_max_range = central+(iter+1)*interval/iter_start_optimization_;
      new_task_min_range = central-(iter+1)*interval/iter_start_optimization_;
      // Distribution
      std::uniform_real_distribution<double> new_distribution (new_task_min_range,
                                                               new_task_max_range);
      ret[i] = new_distribution(random_eng_);
    }
  }
  else{
    //fix the task space range
    currently_extend_task_space_ = false;
    for (int i = 0; i < task_dim_; i++) {
      ret[i] = distribution_[i](random_eng_);
    }
  }
  return ret;
}


MediateTasksGenerator::MediateTasksGenerator(int N_sample,
  int task_dim){
  N_sample_ = N_sample;
  task_dim_ = task_dim;
  start_finding_mediate_sample_ = false;
  currently_find_mediate_sample_ = false;
  choose_sample_from_iter_to_help_ = true;
  sample_index_to_help_= -1;
}

vector<double> MediateTasksGenerator::NewTask(string dir,int iter,
    int sample_idx) {
  DRAKE_DEMAND(start_finding_mediate_sample_);
  // set the task for mediate iteration
  vector<double> ret(task_dim_, 0);

  // get the sample that needs help
  string failed_task_name = to_string(iter) + "_mediate_failed_task.csv";
  string successful_task_name = to_string(iter) + "_mediate_successful_task.csv";
  VectorXd failed_task = readCSV(dir + failed_task_name);
  VectorXd closest_successful_task = readCSV(dir + failed_task_name);

  // uniformly set the tasks
  VectorXd new_task = closest_successful_task+sample_idx*
      (failed_task-closest_successful_task)/(N_sample_-1);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = new_task[i];
  }

  return ret;
}

void MediateTasksGenerator::set_mediate_samples(string dir,int iter,
    const TasksGenerator* task_gen){
  DRAKE_DEMAND(start_finding_mediate_sample_);
  // set the task for mediate iteration
  int is_success;
  string prefix;
  VectorXd failed_task;
  VectorXd closest_successful_task;
  int sample_num = 0;
  string prefix_closest_task;

  //find the sample to help
  for (int sample_num = 0; sample_num < task_gen->total_sample_number();
       sample_num++){
    prefix = to_string(iter)+ string("_") + to_string(sample_num);
    int is_success = (readCSV(dir + prefix +
        string("_is_success.csv")))(0, 0);
    if(is_success==0)
    {
      sample_index_to_help_ = sample_num;
      break;
    }
  }
  prefix = to_string(iter) + string("_") + to_string(sample_index_to_help_);
  failed_task = readCSV(dir + prefix +string("_task.csv"));
  // tasks are uniformly chosen from the range of closest successful sample
  // to failed sample
  int total_sample_number = task_gen->total_sample_number();
  if(choose_sample_from_iter_to_help_){
    // the closest successful sample is chosen from the normal tasks

    prefix_closest_task = to_string(iter)
        + string("_") + to_string(0);
    // find the closest sample
    VectorXd task_scale = task_gen->GetTaskScale();
    for (sample_num = 0; sample_num < total_sample_number;sample_num++) {
      prefix = to_string(iter) + string("_") + to_string(sample_num);
      prefix_closest_task = CompareTwoTasks(dir,prefix_closest_task,
                                            prefix,failed_task,task_scale);
    }
    closest_successful_task = readCSV(dir + prefix_closest_task
                                          +string("_task.csv"));
  }
  else{
    // the closest successful sample is chosen from the mediate samples
    for (sample_num = N_sample_-1+total_sample_number;
         sample_num >= total_sample_number; sample_num--){
      prefix_closest_task = to_string(iter) + string("_") +
          to_string(sample_num);
      is_success = (readCSV(dir + prefix_closest_task +
          string("_is_success.csv")))(0, 0);
      if(is_success==1)
      {
        closest_successful_task = readCSV(dir + prefix_closest_task
                                              +string("_task.csv"));
        break;
      }
    }
  }
  cout<<"sample index to help: "<<sample_index_to_help_<<endl;
  cout<<"closest sample prefix: "<<prefix_closest_task<<endl;
  //the sample to help is required to be a successful sample
  is_success = (readCSV(dir + prefix_closest_task +
      string("_is_success.csv")))(0, 0);
  DRAKE_DEMAND(is_success==1);
  // save the failed task, successful task and
  // solution of this closest sample as initial guess for mediate samples
  VectorXd initial_guess = readCSV(dir + prefix_closest_task
                                       +string("_w.csv"));
  string initial_file_name = to_string(iter) + "_mediate_initial_guess.csv";
  string failed_task_name = to_string(iter) + "_mediate_failed_task.csv";
  string successful_task_name = to_string(iter) + "_mediate_successful_task.csv";
  writeCSV(dir + initial_file_name, initial_guess);
  writeCSV(dir + failed_task_name, failed_task);
  writeCSV(dir + successful_task_name, closest_successful_task);
}

// calculate the third power of L3 norm between two tasks
double TaskDistanceCalculation(const VectorXd& past_task,
                                const VectorXd& current_task,
                                const VectorXd& task_scale){
  VectorXd dif_task =
      (past_task - current_task).array().abs() * task_scale.array();
  VectorXd dif_task2 = dif_task.array().pow(2);
  double distance_task = (dif_task.transpose() * dif_task2)(0, 0);

  return distance_task;
}

// calculate the distance between two past tasks with current tasks and return
// the prefix of the closer task
string CompareTwoTasks(const string& dir, string prefix1,string prefix2,
                       const VectorXd& current_task,
                       const VectorXd& task_scale){
  string prefix_closer_task = prefix1;
  int is_success = (readCSV(dir + prefix2 + string("_is_success.csv")))(0, 0);
  if (is_success == 1){
    is_success = (readCSV(dir + prefix1 + string("_is_success.csv")))(0, 0);
    if(is_success == 0)
    {
      prefix_closer_task = prefix2;
    }
    else{
      // extract past tasks
      VectorXd past_task1 = readCSV(dir + prefix1 + string("_task.csv"));
      VectorXd past_task2 = readCSV(dir + prefix2 + string("_task.csv"));
      // calculate the distance between two tasks
      double distance_task1 = TaskDistanceCalculation(past_task1,
                                                        current_task,task_scale);
      double distance_task2 = TaskDistanceCalculation(past_task2,
                                                        current_task,task_scale);
      if(distance_task2<distance_task1){
        prefix_closer_task = prefix2;
      }
    }
  }
  return prefix_closer_task;
}

}  // namespace goldilocks_models
}  // namespace dairlib
