//
// Created by jianshu on 4/8/20.
//
#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <sys/stat.h>  // Check the existence of a file/folder
#include <cstdlib>  // System call to create folder (and also parent directory)


#include "common/find_resource.h"
#include "examples/goldilocks_models/initial_guess.h"
#include "systems/goldilocks_models/file_utils.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using dairlib::FindResourceOrThrow;


namespace dairlib::goldilocks_models {
    int test_initial_guess(int argc, char* argv[]){
      //create test data and save it
      int iter = 10;
      int sample = 0;
      int robot = 1;
      int use_database = false;
      GridTasksGenerator task_gen;
      if (robot == 0) {
        task_gen = GridTasksGenerator(
            3, {"stride length", "ground incline", "velocity"},
            {10, 5, 5}, {0.25, 0, 0.4},
            {0.015, 0.05, 0.02}, true);
      } else{
        task_gen = GridTasksGenerator(
            4, {"stride length", "ground incline", "velocity", "turning rate"},
            {10, 5, 5, 5},
            {0.3, 0, 0.5, 0}, {0.015, 0.05, 0.04, 0.125}, true);
      }
      int total_sample_num = task_gen.total_sample_number();
      double min_sl = task_gen.task_min("stride_length");
      double max_sl = task_gen.task_max("stride_length");
      double min_gi = task_gen.task_min("ground_incline");
      double max_gi = task_gen.task_max("ground_incline");
      double min_tr = task_gen.task_min("turning_rate");
      double max_tr = task_gen.task_max("turning_rate");
      const string dir = "../dairlib_data/goldilocks_models/find_models/robot_1_test/";

      bool use_created_data = true;
      if(use_created_data) {
        //for each iteration, create theta_s and theta_sDDot
        int iteration = 0;
        for (iteration = 0; iteration <= iter; iteration++) {
          VectorXd theta_s = VectorXd::Random(70);
          VectorXd theta_sDDot = VectorXd::Random(7);
          writeCSV(dir + to_string(iteration) + string("_theta_s.csv"),
                   theta_s);
          writeCSV(dir + to_string(iteration) + string("_theta_sDDot.csv"),
                   theta_sDDot);
          //for each sample, create gamma, is_success and w
          int sample = 0;
          for (sample = 0; sample <= total_sample_num; sample++) {
            string
                prefix = to_string(iteration) + "_" + to_string(sample) + "_";
            MatrixXd
                stride_length = (max_sl + min_sl) / 2 * MatrixXd::Ones(1, 1) +
                (max_sl - min_sl) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("stride_length.csv"),
                     stride_length);
            MatrixXd
                ground_incline = (max_gi + min_gi) / 2 * MatrixXd::Ones(1, 1)
                + (max_gi - min_gi) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("ground_incline.csv"),
                     ground_incline);
            MatrixXd turning_rate = (max_tr + min_tr) / 2 * MatrixXd::Ones(1, 1)
                + (max_tr - min_tr) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("turning_rate.csv"),
                     turning_rate);
            bool is_success = 1;
            writeCSV(dir + prefix + string("is_success.csv"),
                     is_success * MatrixXd::Ones(1, 1));
            VectorXd w = VectorXd::Random(1478);
            writeCSV(dir + prefix + string("w.csv"), w);
          }
        }
      }
      string initial_file = set_initial_guess(dir, iter, sample,task_gen,
          use_database,robot);
      return 0;
    }
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::test_initial_guess(argc, argv);
}