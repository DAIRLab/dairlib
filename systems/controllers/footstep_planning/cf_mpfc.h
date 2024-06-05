#pragma once
#include <array>
#include <chrono>
#include <map>

#include "cf_mpfc_utils.h"
#include "geometry/convex_polygon_set.h"
#include "solvers/optimization_utils.h"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/gurobi_solver.h"


namespace dairlib {
namespace systems {
namespace controllers {

struct cf_mpfc_solution {
  std::vector<Eigen::VectorXd> xc;
  std::vector<Eigen::Vector4d> xs;
};

class CFMPFC {

};


}
}
}