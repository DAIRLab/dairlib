#include <memory>	
#include <Eigen/Dense>
#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
//#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
//#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/common/trajectories/piecewise_polynomial.h"


#include "examples/Cassie/cassie_utils.h"

namespace dairlib{

using drake::trajectories::PiecewisePolynomial;

PiecewisePolynomial<double> getSimulationTrajectoryOverTime(PiecewisePolynomial<double> traj_input, double simulationTime);
drake::VectorX<double> getDerivativePredictionAtTime(double time, PiecewisePolynomial<double> trajectory, Eigen::Matrix<double, 32, 1> x);

}	//Namespace dairlib