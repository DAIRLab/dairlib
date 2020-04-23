#include "multibody/multipose_visualizer.h"
#include "common/find_resource.h"

int main(int argc, char* argv[]) { 
  std::srand((unsigned int) time(0));
  auto visualizer = dairlib::multibody::MultiposeVisualizer(
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"), 8, "pelvis");

  Eigen::MatrixXd poses = Eigen::MatrixXd::Random(30, 8);
  visualizer.DrawPoses(poses);
}