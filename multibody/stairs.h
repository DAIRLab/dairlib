#pragma once

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"
#include "boxy_height_map.h"

namespace dairlib::multibody {

enum StairDirection {
  kUp = 0,
  kDown = 1
};

class Stairs {
 public:
  Stairs(double width, double depth, double height, double mu, int n,
         bool add_walls, StairDirection stair_direction) :
         width_(width),
         depth_(depth),
         height_(height),
         mu_(mu),
         n_(n),
         add_walls_(add_walls),
         stair_direction_(stair_direction){}

  void AddToPlant(drake::multibody::MultibodyPlant<double>* plant,
                  drake::geometry::SceneGraph<double>* scene_graph);
  static Stairs MakeRandomMap(double mu);

 private:
  double width_;
  double depth_;
  double height_;
  double mu_;
  int n_;
  bool add_walls_;
  StairDirection stair_direction_;
};
}