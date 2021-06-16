//
// Created by brian on 6/15/21.
//
#include "examples/KoopmanMPC/Cassie/cassie_model_utils.h"
#include<gflags/gflags.h>

namespace dairlib{

DEFINE_double(height, 0.8, "Pelvis height for initial pose");

void printCassieParamsMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<std::string> links = {"pelvis", "yaw_left", "yaw_right", "hip_left", "hip_right",
                                    "thigh_left", "thigh_right", "knee_left", "knee_right", "shin_left", "shin_right"};

  PrintCassieSingleRigidBodyParameters(FLAGS_height, links);
}
}

int main(int argc, char* argv[]){
  dairlib::printCassieParamsMain(argc, argv);
}
