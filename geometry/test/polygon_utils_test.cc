#include <chrono>
#include <gflags/gflags.h>
#include "geometry/polygon_utils.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

DEFINE_string(
    bagpath,
    "/media/brian/tb2/cassie_backup/logs/cassie_hardware/2023/04_28_23/"
    "cassie-rosbag-00_2023-04-28-11-19-50.bag", "path to example rosbag"
);

using convex_plane_decomposition_msgs::PlanarTerrain;

namespace dairlib::geometry {

int polytest_main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  rosbag::Bag bag;
  bag.open(FLAGS_bagpath, rosbag::bagmode::Read);
  rosbag::View view(
      bag,
      rosbag::TopicQuery("/convex_plane_decomposition_ros/planar_terrain")
  );

  int i = 0;
  for (const auto &m : view) {
    const PlanarTerrain::ConstPtr terrain = m.instantiate<PlanarTerrain>();
    if (terrain != nullptr) {
      std::cout << i << ": ";
      auto begin = std::chrono::high_resolution_clock::now();
      auto footholds = DecomposeTerrain(*terrain, 0.15);
      auto end = std::chrono::high_resolution_clock::now();
      std::cout
          << static_cast<std::chrono::duration<double>>(end - begin).count()
          << std::endl;
      i++;
    }
  }
  return 0;
}
}

int main(int argc, char **argv) {
  return dairlib::geometry::polytest_main(argc, argv);
}