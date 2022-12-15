
#pragma once

#include <Eigen/Core>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/kinematic_centroidal_planner/cassie_kinematic_centroidal_solver.h"
#include "multibody/kinematic/world_point_evaluator.h"

/*!
 * @brief Construct a nominal stand for cassie
 * @param plant cassie plant
 * @param com_height
 * @param stance_width
 * @param visualize true if the stand should be visualized
 * @return vector of state [nq + nv]
 */
Eigen::VectorXd GenerateNominalStand(
    const drake::multibody::MultibodyPlant<double>& plant, double com_height,
    double stance_width, bool visualize = false);

std::vector<Complexity> GenerateComplexitySchedule(
    int n_knot_points, const std::vector<std::string>& complexity_string_list);