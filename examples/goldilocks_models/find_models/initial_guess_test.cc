#include <gtest/gtest.h>

#include "examples/goldilocks_models/find_models/initial_guess.h"

namespace dairlib::goldilocks_models {

class InitialGuessTest : public ::testing::Test {};

int test_initial_guess(int iter, int sample, int robot) {
  // create test data and save it
  bool use_database = false;
  // create task_gen
  GridTasksGenerator task_gen_grid;
  if (robot == 0) {
    task_gen_grid = GridTasksGenerator(
        3, {"stride length", "ground incline", "velocity"}, {10, 5, 5},
        {0.25, 0, 0.4}, {0.015, 0.05, 0.02}, true);
  } else {
    task_gen_grid = GridTasksGenerator(
        4, {"stride length", "ground incline", "velocity", "turning rate"},
        {10, 5, 5, 5}, {0.3, 0, 0.5, 0}, {0.015, 0.05, 0.04, 0.125}, true);
  }
  TasksGenerator* task_gen = &task_gen_grid;
  int total_sample_num = task_gen->total_sample_number();
  // create task
  Task task(task_gen->names());
  task.set(task_gen->NewTask(sample));
  // create rom
  RomData rom = RomData(1, 2, 2, 2);
  //make sure difference between thetas within theta_range
  VectorXd current_theta = VectorXd::Random(4)/10000+VectorXd::Ones(4);
  rom.SetTheta(current_theta);

  const string dir =
      "../dairlib_data/goldilocks_models/find_models/robot_1_test/";
  //create folder if it doesn't exist
  if(!folder_exist(dir)){
    cout<<"Test folder doesn't exist"<<endl;
    std::string string_for_system_call = "mkdir -p " + dir;
    if (system(string_for_system_call.c_str()) == -1) {
      printf("Error creating directory!n");
      return 0;
    } else {
      cout << "Test folder has been created: " << dir << endl;
    }
  }

  // for each iteration, create theta_s and theta_sDDot
  int iteration = 0;
  for (iteration = 0; iteration <= iter; iteration++) {
    //make sure difference between thetas within theta_range
    VectorXd theta_y = VectorXd::Ones(2)+VectorXd::Random(2)/10000;
    VectorXd theta_yddot = VectorXd::Ones(2)+VectorXd::Random(2)/10000;
    writeCSV(dir + to_string(iteration) + string("_theta_y.csv"), theta_y);
    writeCSV(dir + to_string(iteration) + string("_theta_yddot.csv"),
             theta_yddot);
    // for each sample, create gamma, is_success and w
    int sample = 0;
    int dim = 0;
    for (sample = 0; sample <= total_sample_num; sample++) {
      string prefix = to_string(iteration) + "_" + to_string(sample) + "_";
      VectorXd gamma(task_gen->dim());
      for (dim = 0; dim < task_gen->dim(); dim++) {
        double min = task_gen->task_min(task_gen->names()[dim]);
        double max = task_gen->task_min(task_gen->names()[dim]);
        std::uniform_real_distribution<double> dist(min, max);
        std::default_random_engine re;
        gamma[dim] = dist(re);
      }
      writeCSV(dir + prefix + string("task.csv"), gamma);
      int is_success = 1;
      writeCSV(dir + prefix + string("is_success.csv"),
               is_success * MatrixXd::Ones(1, 1));
      VectorXd w = VectorXd::Random(20);
      writeCSV(dir + prefix + string("w.csv"), w);
    }
  }
  string initial_file = SetInitialGuessByInterpolation(
      dir, iter, sample, task_gen, task, rom, use_database, robot);
  return 1;
}

TEST_F(InitialGuessTest, DifferentIter) {
  EXPECT_EQ(1, test_initial_guess(10, 0, 0));
  EXPECT_EQ(1, test_initial_guess(15, 0, 0));
  EXPECT_EQ(1, test_initial_guess(20, 0, 0));
}
TEST_F(InitialGuessTest, DifferentRobot) {
  EXPECT_EQ(1, test_initial_guess(10, 0, 1));
  EXPECT_EQ(1, test_initial_guess(15, 0, 1));
  EXPECT_EQ(1, test_initial_guess(20, 0, 1));
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
