#include "examples/goldilocks_models/task.h"

#include <iostream>

//this should be deleted later
namespace dairlib::goldilocks_models {
int test(int argc, char *argv[]) {
  const string dir = "../dairlib_data/goldilocks_models/find_models/robot_" +
      to_string(1) + "/";
  int sample_num = 0;
  int is_success;

  string prefix = to_string(132) + string("_") +to_string(0);
  string prefix_closest_task = to_string(132) + string("_") +to_string(0);
  VectorXd failed_task = readCSV(dir + prefix +string("_task.csv"));
  VectorXd task_scale = VectorXd::Ones(4);
  for (sample_num = 0; sample_num < 16; sample_num++) {
    prefix = to_string(132) + string("_") + to_string(sample_num);
    prefix_closest_task = CompareTwoTasks(dir,prefix_closest_task,
                                         prefix,failed_task,task_scale);
  }
  cout<<prefix_closest_task<<endl;
  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::test(argc, argv);
}