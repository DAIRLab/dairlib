//
// Created by brian on 6/15/21.
//

#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/context.h"

namespace dairlib{

using Eigen::VectorXd;
using Eigen::Vector3d;

using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::RotationalInertia;


void PrintCassieSingleRigidBodyParameters(double h_des, std::vector<std::string> link_names) {

  // Load Cassie from urdf and find fixed point
  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr,
                     true, urdf, true, true);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();


  VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0;
  double min_norml_fp = 70;
  double toe_spread = 0.125;

  CassieFixedPointSolver(plant, h_des, mu_fp, min_norml_fp,
                         true, toe_spread, &q_init, &u_init, &lambda_init);

  plant.SetPositions(plant_context.get(), q_init);

  RotationalInertia I = multibody::CalcLinkInertiaAboutPlantCom(plant, *plant_context,
      plant.world_frame(), link_names[0]);

  double mass = plant.GetBodyByName(link_names[0]).get_mass(*plant_context);

  for (int i = 1; i < link_names.size(); i ++) {
    mass += plant.GetBodyByName(link_names[i]).get_mass(*plant_context);
    I += multibody::CalcLinkInertiaAboutPlantCom(plant, *plant_context, plant.world_frame(), link_names[i]);
  }

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);

  // Get body frames and points
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant.GetFrameByName("toe_right"));

  Vector3d left_pos;
  Vector3d right_pos;
  Vector3d com_pos = plant.CalcCenterOfMassPositionInWorld(*plant_context);

  plant.CalcPointsPositions(*plant_context, left_toe_mid.second,
      left_toe_mid.first, plant.world_frame(), &left_pos);
  plant.CalcPointsPositions(*plant_context, right_toe_mid.second,
                            right_toe_mid.first, plant.world_frame(), &right_pos);

  std::cout << "Com Position:\n" << com_pos << "\nLeft Foot Pos\n" << left_pos
            << "\nRight Foot Pos\n" << right_pos << std::endl;
  std::cout << "Rotational Inertia:\n" << I << "\n\nMass: " << mass << std::endl;
}
}