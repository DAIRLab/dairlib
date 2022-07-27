#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib::systems::controllers::alip_utils {

typedef std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&> PointOnFramed;

void CalcAlipState(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const Eigen::VectorXd &x,
                   const std::vector<PointOnFramed> stance_locations,
                   const drake::EigenPtr<Eigen::Vector3d> &CoM_p,
                   const drake::EigenPtr<Eigen::Vector3d> &L_p,
                   const drake::EigenPtr<Eigen::Vector3d> &stance_pos_p);

Eigen::Matrix4d CalcA(double com_z, double m);

double XImpactTime(double t_start, double H, double m, double x, double Ly,
                   double x_impact);

double YImpactTime(double t_start, double H, double m, double y, double Lx,
                   double y_impact);

}