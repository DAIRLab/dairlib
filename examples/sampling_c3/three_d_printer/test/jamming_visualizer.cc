// Meshcat view of a jamming sweep: the sampled EE locations, colored by each
// predicted-force metric, in the actual cone/ramp/printer scene.
//
// The matplotlib output from plot_jamming_sweep.py flattens a 3D question onto
// 2D axes -- the kRandomOnShell draws wrap around the cone in three dimensions,
// so a histogram cannot show *where* a hot sample sits relative to the cone,
// the ramp, and the keep-out box.  This draws the force field in place.
//
// Each metric becomes its own point cloud at its own meshcat path under
// "jamming/", which is what gives it a visibility checkbox in meshcat's scene
// tree -- the same idiom three_d_printer_visualizer.cc uses to toggle keep-out
// regions per goal.
//
// Colors run green to red via the shared RdYlGn-reversed colormap, and what
// green MEANS is declared per column in MeaningOf below rather than left to a
// convention.  Three labels, because no single one is honest for all of them:
//
//   green = low jam risk   most columns.  Usually their low end -- less force,
//                          less tracking error -- but the HIGH end for object
//                          travel and progress, since the object moving is
//                          exactly what a jam prevents.
//   green = low cost       c3_cost alone.  An objective, not a risk; in this
//                          scene the cheapest samples are the ones that jam, so
//                          it goes green where the risk columns go red.
//   green = low value      diagnostics.  The ramp still has to point somewhere,
//                          and this says it encodes magnitude and nothing else.
//
// Three columns are computed here rather than read from the file:
// ee_tracking_frac, log_u_per_point_travel, and the jam_risk_general score
// built from them.  See AppendDerivedMetrics for what they mean and where their
// weights came from.
//
// Both surfaces carry the label: a "green =" column in the printed scale table,
// and the same tag in the meshcat path, since the Controls -> Scene tree is the
// only text the 3D view has.
//
// Samples whose C3 solve fell back to zero inputs are drawn BLACK, and samples
// whose solve produced no plan to execute are drawn GREY; both understate the
// force and would otherwise masquerade as safe low-force locations.
//
// The color endpoints for every metric are printed to stdout, so the visual
// pattern can be read back in Newtons (or meters, for the tracking error).
//
// Usage:
//   bazel-bin/.../jamming_visualizer --sweep_file
//   /tmp/jam/jamming_sweep_random.txt

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/visualizer_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/senders/cost_colormap.h"
#include "systems/visualization/static_visualization_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/perception/point_cloud.h"
#include "drake/perception/point_cloud_flags.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace {

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::perception::PointCloud;
using drake::systems::DiagramBuilder;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;
using std::vector;

DEFINE_string(demo_name, "cone", "Demo within sampling_c3/three_d_printer/.");
DEFINE_string(sweep_file, "/tmp/jam/jamming_sweep_random.txt",
              "The jamming_sweep output file to visualize.");
DEFINE_double(point_size, -1.0,
              "Rendered size of each sample point; negative means use "
              "vis_params.sample_buffer_point_size.");
DEFINE_string(printer_joints, "0.35,0.35,0.345",
              "Printer x,y,z joint positions.  The default parks the gantry at "
              "the far +y corner and the top, clear of the cone and ramp.");
DEFINE_string(color_min, "",
              "Per-metric overrides for the LOW end of the value range, as "
              "'metric:value[,metric:value...]'.  Unset metrics use the 2nd "
              "percentile over successfully-solved samples.  Whether the low "
              "end is drawn green or red follows the metric's direction, so "
              "these stay value bounds rather than color bounds.");
DEFINE_string(color_max, "",
              "Per-metric overrides for the HIGH end of the value range, same "
              "format as --color_min.  Unset metrics use the 98th percentile.");
DEFINE_double(color_low_percentile, 2.0,
              "Percentile taken as the low end of the value range.");
DEFINE_double(color_high_percentile, 98.0,
              "Percentile taken as the high end of the value range.");
DEFINE_string(slice_by, "",
              "Draw only the samples whose value of this column falls in "
              "--slice_range, and take the color percentiles from just those.  "
              "A viewing aid for a dense cloud -- e.g. --slice_by=ee_y "
              "--slice_range=0.06,0.08 to read a metric at fixed azimuth.  "
              "Nothing computed elsewhere slices.");
DEFINE_string(slice_range, "", "'lo,hi' bounds for --slice_by, inclusive.");

// One sweep file: the frozen scene it was produced from, plus a column-indexed
// table of samples.
struct Sweep {
  Vector3d object_position = Vector3d::Zero();
  Vector4d object_orientation = Vector4d(1, 0, 0, 0);
  int goal_step = 0;
  // The three C3 states the sweep solved against, as x_lcs vectors
  // [ee_xyz, object_quat, object_xyz, velocities].  Empty if the file predates
  // the c3_state_* header lines.
  VectorXd x_actual;
  VectorXd x_target;
  VectorXd x_final_target;
  vector<string> column_names;
  // data[column][sample]
  vector<vector<double>> data;

  int num_samples() const { return data.empty() ? 0 : data.front().size(); }

  const vector<double>& Column(const string& name) const {
    for (size_t i = 0; i < column_names.size(); ++i) {
      if (column_names[i] == name) {
        return data.at(i);
      }
    }
    throw std::runtime_error("Sweep file has no column '" + name + "'");
  }

  bool HasColumn(const string& name) const {
    return std::find(column_names.begin(), column_names.end(), name) !=
           column_names.end();
  }
};

Sweep ReadSweep(const string& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Could not open " + path);
  }
  Sweep sweep;
  string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      std::istringstream stream(line.substr(1));
      vector<string> tokens;
      string token;
      while (stream >> token) {
        tokens.push_back(token);
      }
      // "# <name> v0 v1 ..." -> the values as a vector.
      auto values_after_name = [&tokens]() {
        VectorXd values(tokens.size() - 1);
        for (size_t i = 1; i < tokens.size(); ++i) {
          values(i - 1) = std::stod(tokens[i]);
        }
        return values;
      };
      if (tokens.empty()) {
        continue;
      }
      if (tokens.front() == "c3_state_actual") {
        sweep.x_actual = values_after_name();
      } else if (tokens.front() == "c3_state_target") {
        sweep.x_target = values_after_name();
      } else if (tokens.front() == "c3_state_final_target") {
        sweep.x_final_target = values_after_name();
      } else if (tokens.front() == "scene") {
        // "# scene object_xyz x y z object_quat w x y z goal_step n"
        for (size_t i = 0; i < tokens.size(); ++i) {
          if (tokens[i] == "object_xyz" && i + 3 < tokens.size()) {
            sweep.object_position =
                Vector3d(std::stod(tokens[i + 1]), std::stod(tokens[i + 2]),
                         std::stod(tokens[i + 3]));
          } else if (tokens[i] == "object_quat" && i + 4 < tokens.size()) {
            sweep.object_orientation =
                Vector4d(std::stod(tokens[i + 1]), std::stod(tokens[i + 2]),
                         std::stod(tokens[i + 3]), std::stod(tokens[i + 4]));
          } else if (tokens[i] == "goal_step" && i + 1 < tokens.size()) {
            sweep.goal_step = std::stoi(tokens[i + 1]);
          }
        }
      } else if (tokens.front() == "ee_x" && sweep.column_names.empty()) {
        // Several comment lines precede the header, so identify it by the
        // column it must start with rather than by position.
        sweep.column_names = tokens;
        sweep.data.resize(tokens.size());
      }
      continue;
    }
    if (sweep.column_names.empty()) {
      throw std::runtime_error(path + " has data before its column header");
    }
    std::istringstream stream(line);
    for (size_t i = 0; i < sweep.column_names.size(); ++i) {
      // Read as a token and convert by hand: operator>>(double) fails on the
      // "nan" that unavailable metrics are written as, and C++11 then stores 0
      // -- which would silently turn a missing metric into a zero force.
      string token;
      double value = std::numeric_limits<double>::quiet_NaN();
      if (stream >> token) {
        try {
          value = std::stod(token);
        } catch (const std::exception&) {
          value = std::numeric_limits<double>::quiet_NaN();
        }
      }
      sweep.data.at(i).push_back(value);
    }
  }
  if (sweep.num_samples() == 0) {
    throw std::runtime_error(path + " contains no samples");
  }
  return sweep;
}

// The name a quantity goes by in this sweep: its own, or the "_retimed" spelling
// older files used before the unretimed family was dropped.  Empty if neither.
string ResolveColumn(const Sweep& sweep, const string& name) {
  if (sweep.HasColumn(name)) {
    return name;
  }
  const string legacy = name + "_retimed";
  return sweep.HasColumn(legacy) ? legacy : string();
}

void AppendColumn(Sweep* sweep, const string& name, vector<double> values) {
  sweep->column_names.push_back(name);
  sweep->data.push_back(std::move(values));
}

// The distance from the cone's body origin to its outermost collision sphere,
// read off cone.sdf (the apex, at 0.0494 m; the base spheres sit at 0.0254 m).
// It is what lets rotation be counted as motion without inventing a constant:
// the object's own geometry supplies the length that turns radians into meters.
constexpr double kObjectRadius = 0.0494;

// Guards for the two ratios.  A plan that asks for nothing, or an object that
// goes nowhere, would otherwise divide by zero and paint an infinity.
constexpr double kMinDisplacement = 1e-4;   // meters
constexpr double kMinPointTravel = 1e-4;    // meters
constexpr double kMinEffortRatio = 1e-9;    // newtons per meter, before log10

// One term of the jamming score: a value, the standardization that puts it on a
// common scale, and its weight.
struct RiskTerm {
  const char* name;
  double mean;
  double std_dev;
  double coefficient;
};

// The three-term score, and the two ratios it is built from.
//
// Both ratios are frame-invariant by construction: neither names an axis, and
// neither says which way the object ought to move.  That is the point of them.
// The terms they replace -- the z component of the tracking error, and the
// object's rotation read as a risk -- score better on the goal 1 -> 2 leg and
// worse elsewhere on the shell, because they encode this cone tipping against
// this ramp rather than a mechanism.  Split into blocks of the sampled shell,
// rotation-as-risk keeps its sign in only 2 blocks of 6; every term below keeps
// it in 5 or 6.
//
//   plan_ee_displacement    whether the retimed plan asks the end effector to
//                           go anywhere at all.
//   ee_tracking_frac        max_delta_x_ee_norm / plan_ee_displacement: how far
//                           the PD rollout diverges from the plan, per meter of
//                           plan.  The direction-free replacement for
//                           max_delta_x_ee_z, and better than it on both counts.
//   log_u_per_point_travel  log10 of the commanded effort per meter the object's
//                           furthest material point moves.  Rotation enters the
//                           denominator as motion rather than as risk.
//
// None of the three is worth thresholding alone -- the effort ratio is at chance
// marginally, and the displacement's sign flips between the near and far faces.
// They only separate jointly, which is why the score exists as a column instead
// of as advice to look at three others.
//
// The weights are a logistic fit against the ground-truth jammed labels on
// /tmp/jam/jamming_sweep_random.txt (1685 real plans, 57% jammed), and the
// standardization is that file's own means and standard deviations.  Both are
// frozen here rather than refit per file, so the score is one fixed function of
// a sample: drawing it over a *different* sweep is an honest out-of-sample test.
// Drawing it over the file it was fit on is not, and DoMain says so.
void AppendDerivedMetrics(Sweep* sweep) {
  const string displacement_name = ResolveColumn(*sweep, "plan_ee_displacement");
  const string tracking_name = ResolveColumn(*sweep, "max_delta_x_ee_norm");
  const string effort_name = ResolveColumn(*sweep, "max_u_norm_c3");
  const string travel_name = ResolveColumn(*sweep, "object_travel_in_rollout");
  const string rotation_name =
      ResolveColumn(*sweep, "object_rotation_in_rollout");
  vector<string> missing;
  for (const auto& [label, resolved] :
       {std::pair<const char*, const string&>{"plan_ee_displacement",
                                              displacement_name},
        {"max_delta_x_ee_norm", tracking_name},
        {"max_u_norm_c3", effort_name},
        {"object_travel_in_rollout", travel_name},
        {"object_rotation_in_rollout", rotation_name}}) {
    if (resolved.empty()) {
      missing.push_back(label);
    }
  }
  if (!missing.empty()) {
    std::cout << "Not computing jam_risk_general: this sweep has no";
    for (const string& name : missing) {
      std::cout << " " << name;
    }
    std::cout << (missing.size() == 1 ? " column." : " columns.") << std::endl;
    return;
  }

  static const RiskTerm kTerms[] = {
      {"plan_ee_displacement", 0.031420013867011265, 0.02658335827932383,
       3.1487515059470774},
      {"ee_tracking_frac", 3.7510795656163043, 2.69499669197729,
       2.9073338702683964},
      {"log_u_per_point_travel", 2.617961796894917, 0.5372553147919489,
       2.3565367547080616},
  };
  constexpr double kIntercept = 1.0153694534039608;

  const vector<double>& displacement = sweep->Column(displacement_name);
  const vector<double>& tracking = sweep->Column(tracking_name);
  const vector<double>& effort = sweep->Column(effort_name);
  const vector<double>& travel = sweep->Column(travel_name);
  const vector<double>& rotation = sweep->Column(rotation_name);

  const int num_samples = sweep->num_samples();
  vector<double> tracking_frac(num_samples);
  vector<double> effort_per_travel(num_samples);
  vector<double> risk(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    // Rotation counted as motion, scaled by the object's own radius: an upper
    // bound on how far its furthest material point travels.
    const double point_travel = travel[i] + kObjectRadius * rotation[i];
    tracking_frac[i] =
        tracking[i] / std::max(displacement[i], kMinDisplacement);
    effort_per_travel[i] = std::log10(std::max(
        effort[i] / std::max(point_travel, kMinPointTravel), kMinEffortRatio));
    const double values[] = {displacement[i], tracking_frac[i],
                             effort_per_travel[i]};
    double score = kIntercept;
    for (int t = 0; t < 3; ++t) {
      score += kTerms[t].coefficient * (values[t] - kTerms[t].mean) /
               kTerms[t].std_dev;
    }
    // A NaN in any input carries through, and the sample is drawn black --
    // which is what an unmeasurable risk should look like.
    risk[i] = score;
  }
  AppendColumn(sweep, "ee_tracking_frac", std::move(tracking_frac));
  AppendColumn(sweep, "log_u_per_point_travel", std::move(effort_per_travel));
  AppendColumn(sweep, "jam_risk_general", std::move(risk));
  std::cout << "Computed jam_risk_general from " << displacement_name << ", "
            << tracking_name << ", " << effort_name << ", " << travel_name
            << " and " << rotation_name
            << ", along with the two ratios it is built from." << std::endl;
}

// Parses "metric:value,metric:value" into a lookup.
std::map<string, double> ParseOverrides(const string& text) {
  std::map<string, double> overrides;
  if (text.empty()) {
    return overrides;
  }
  std::istringstream stream(text);
  string entry;
  while (std::getline(stream, entry, ',')) {
    const size_t colon = entry.find(':');
    if (colon == string::npos) {
      throw std::runtime_error("Expected 'metric:value', got '" + entry + "'");
    }
    overrides[entry.substr(0, colon)] = std::stod(entry.substr(colon + 1));
  }
  return overrides;
}

// Linear interpolation into a sorted sample, matching numpy.percentile.
double Percentile(vector<double> sorted_values, double percentile) {
  if (sorted_values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(sorted_values.begin(), sorted_values.end());
  const double position = percentile / 100.0 * (sorted_values.size() - 1);
  const int lower = static_cast<int>(std::floor(position));
  const int upper = static_cast<int>(std::ceil(position));
  const double fraction = position - lower;
  return sorted_values[lower] * (1.0 - fraction) +
         sorted_values[upper] * fraction;
}

// Draws a coordinate triad at @p path, in the same three-cylinder style
// LcmC3TargetDrawer uses for C3 states in the live visualizer, so the offline
// view reads the same way.  The caller positions it with SetTransform(path).
void AddTriad(Meshcat* meshcat, const string& path, double length,
              double radius, const drake::geometry::Rgba& color) {
  const drake::geometry::Cylinder cylinder(radius, length);
  const double half = 0.5 * length;
  // A Drake Cylinder runs along its own +z, so each axis is a 90 degree
  // rotation of that, shifted out to start at the origin.
  const std::pair<const char*, RigidTransformd> axes[] = {
      {"x-axis", RigidTransformd(RotationMatrixd(Eigen::AngleAxisd(
                                     0.5 * M_PI, Vector3d::UnitY())),
                                 Vector3d(half, 0.0, 0.0))},
      {"y-axis", RigidTransformd(RotationMatrixd(Eigen::AngleAxisd(
                                     0.5 * M_PI, Vector3d::UnitX())),
                                 Vector3d(0.0, half, 0.0))},
      {"z-axis", RigidTransformd(Vector3d(0.0, 0.0, half))},
  };
  for (const auto& [name, transform] : axes) {
    meshcat->SetObject(path + "/" + name, cylinder, color);
    meshcat->SetTransform(path + "/" + name, transform);
  }
}

// Places a triad at the object pose held in an x_lcs state vector.
void DrawObjectState(Meshcat* meshcat, const string& path,
                     const VectorXd& x_lcs,
                     const drake::geometry::Rgba& color) {
  AddTriad(meshcat, path, /*length=*/0.1, /*radius=*/0.005, color);
  meshcat->SetTransform(
      path,
      RigidTransformd(
          RotationMatrixd(
              Quaterniond(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6)).normalized()),
          Vector3d(x_lcs(7), x_lcs(8), x_lcs(9))));
}

// Places a smaller, axis-aligned triad at the EE position held in an x_lcs
// state vector.  The LCS EE is a point, so only its position is meaningful.
void DrawEEState(Meshcat* meshcat, const string& path, const VectorXd& x_lcs,
                 const drake::geometry::Rgba& color) {
  AddTriad(meshcat, path, /*length=*/0.05, /*radius=*/0.0025, color);
  meshcat->SetTransform(
      path, RigidTransformd(Vector3d(x_lcs(0), x_lcs(1), x_lcs(2))));
}

// What green means for one column.  Three labels, stated outright, because a
// single "green is good" convention cannot cover all of them honestly: most
// columns carry a jamming claim, the cost carries an objective, and several
// carry no claim at all beyond their own magnitude.
enum class ColorMeaning {
  // A jamming-risk column read the natural way: less of it is less risk.
  kJamRiskLowGreen,
  // A jamming-risk column read the other way: the object moving is the thing
  // that keeps it out of a jam, so the HIGH end is the green one.  These are
  // the reason the ramp cannot be oriented by magnitude alone.
  kJamRiskHighGreen,
  // c3_cost.  Low-to-green like most of the risk columns, but kept separate
  // because it is an objective, not a risk: cheap does not mean safe.  In this
  // scene it emphatically does not -- see the note the table prints.
  kCost,
  // A diagnostic: a baseline, a property of the plan, or a quantity nobody has
  // tied to an outcome.  The ramp still has to point somewhere, so it points
  // low-to-green, and the label says green = low value to make plain that the
  // color encodes magnitude and claims nothing else.
  kMagnitude,
  // Rendered exactly like kMagnitude, and labelled the same, but tracked apart
  // so an unrecognised column is named in a warning instead of passing as a
  // considered decision.
  kUnrecognised,
};

// What green means for every column a sweep can carry.
ColorMeaning MeaningOf(const string& metric) {
  static const std::map<string, ColorMeaning>* const kMeanings =
      new std::map<string, ColorMeaning>{
          // The objective.  Its own label, because in this scene it points the
          // opposite way from the risk columns.
          {"c3_cost", ColorMeaning::kCost},

          // Effort and error: every force the plan commands, every newton the
          // world braces back with, every meter the end effector falls behind,
          // and every knot spent pinned against an input bound.  More of any of
          // them is more risk.
          {"max_u_norm_c3", ColorMeaning::kJamRiskLowGreen},
          {"max_u_xy_c3", ColorMeaning::kJamRiskLowGreen},
          {"max_u_z_c3", ColorMeaning::kJamRiskLowGreen},
          {"max_u_norm_pd", ColorMeaning::kJamRiskLowGreen},
          {"max_u_xy_pd", ColorMeaning::kJamRiskLowGreen},
          {"max_u_z_pd", ColorMeaning::kJamRiskLowGreen},
          {"max_u_norm_pd_coarse", ColorMeaning::kJamRiskLowGreen},
          {"max_ee_contact_force_c3", ColorMeaning::kJamRiskLowGreen},
          {"max_ee_contact_force_pd", ColorMeaning::kJamRiskLowGreen},
          {"max_delta_x_ee_norm", ColorMeaning::kJamRiskLowGreen},
          {"max_delta_x_ee_xy", ColorMeaning::kJamRiskLowGreen},
          {"max_delta_x_ee_z", ColorMeaning::kJamRiskLowGreen},
          {"frac_knots_u_xy_at_limit", ColorMeaning::kJamRiskLowGreen},
          {"frac_knots_u_z_at_limit", ColorMeaning::kJamRiskLowGreen},
          {"max_object_ground_force_c3", ColorMeaning::kJamRiskLowGreen},
          {"object_ground_force_per_travel", ColorMeaning::kJamRiskLowGreen},
          {"sim_ee_tracking_error", ColorMeaning::kJamRiskLowGreen},
          // Verdicts, where 1 is the bad outcome.
          {"no_op_plan", ColorMeaning::kJamRiskLowGreen},
          {"solve_fell_back", ColorMeaning::kJamRiskLowGreen},
          {"jammed", ColorMeaning::kJamRiskLowGreen},

          // Progress.  The object going somewhere is what a jam prevents, so
          // these are the columns whose green end is the high one.
          {"object_travel_in_rollout", ColorMeaning::kJamRiskHighGreen},
          {"sim_object_travel", ColorMeaning::kJamRiskHighGreen},
          {"sim_object_progress", ColorMeaning::kJamRiskHighGreen},

          // No claim attached.  How far the object settles with the end
          // effector held still is the baseline sim_object_progress subtracts;
          // how far a plan asked the end effector to travel is a property of
          // the plan; the peak scene contact force is unclassified and includes
          // the object's own weight on the build plate; and a reorientation
          // magnitude cannot tell an intended tip onto the ramp from the cone
          // being knocked over, so it is not evidence either way.
          {"sim_object_travel_passive", ColorMeaning::kMagnitude},
          {"plan_ee_displacement", ColorMeaning::kMagnitude},
          {"sim_max_contact_force", ColorMeaning::kMagnitude},
          {"object_rotation_in_rollout", ColorMeaning::kMagnitude},
          {"sim_object_rotation", ColorMeaning::kMagnitude},
          // The two ratios the score is built from.  Each is direction-free,
          // but neither holds its sign across the whole shell on its own -- the
          // tracking fraction points one way on the far face and the other way
          // marginally -- so neither carries a jamming claim by itself.  The
          // score does; they are here to diagnose it, not to be thresholded.
          {"ee_tracking_frac", ColorMeaning::kMagnitude},
          {"log_u_per_point_travel", ColorMeaning::kMagnitude},

          // The score itself, which is a jamming claim and nothing else.
          {"jam_risk_general", ColorMeaning::kJamRiskLowGreen},
      };

  if (auto it = kMeanings->find(metric); it != kMeanings->end()) {
    return it->second;
  }
  // Sweeps written before the unretimed family was dropped suffix their
  // columns; the quantity, and so its meaning, is unchanged.
  constexpr const char* kLegacySuffix = "_retimed";
  const size_t suffix_length = std::strlen(kLegacySuffix);
  if (metric.size() > suffix_length &&
      metric.compare(metric.size() - suffix_length, suffix_length,
                     kLegacySuffix) == 0) {
    const string base = metric.substr(0, metric.size() - suffix_length);
    if (auto it = kMeanings->find(base); it != kMeanings->end()) {
      return it->second;
    }
  }
  return ColorMeaning::kUnrecognised;
}

// The label both surfaces carry, completing the sentence "green = ...".
const char* GreenMeansLabel(ColorMeaning meaning) {
  switch (meaning) {
    case ColorMeaning::kJamRiskLowGreen:
    case ColorMeaning::kJamRiskHighGreen:
      return "low jam risk";
    case ColorMeaning::kCost:
      return "low cost";
    case ColorMeaning::kMagnitude:
    case ColorMeaning::kUnrecognised:
      return "low value";
  }
  return "low value";
}

// The meshcat path a metric's cloud is published at.  The label rides in the
// name because the Controls -> Scene tree is the only text the 3D view has:
// without it, a viewer toggling between clouds has nothing telling them that
// green just changed meaning.
string MeshcatPath(const string& metric, ColorMeaning meaning) {
  return "jamming/" + metric + " [green = " + GreenMeansLabel(meaning) + "]";
}

// How one metric's values map onto the green-to-red ramp.  Note that `green`
// sits above `red` for a kJamRiskHighGreen column -- the ramp is oriented by
// what the column means, not by magnitude.
struct ColorScale {
  double green = 0.0;  // value rendered pure green
  double red = 1.0;    // value rendered pure red
  double min = 0.0;    // true extremes over solved samples, for reporting
  double median = 0.0;
  double max = 0.0;
  int num_solved = 0;
  ColorMeaning meaning = ColorMeaning::kUnrecognised;
};

// The samples whose value is a measurement of the thing the metric names, and
// so the only ones a color scale may be built from.  A solve that fell back, or
// one that produced no plan to execute, reports a *low* force because nothing
// happened -- painting those green would read as "this is a safe place to push"
// when the truth is "we learned nothing here".
struct DrawnSamples {
  std::vector<int> indices;   // in slice, whatever their verdict
  std::vector<bool> is_real;  // and worth putting on the scale
  int num_failed = 0;
  int num_no_op = 0;
};

// Which samples produced no plan to execute.  Sweeps written since the metric
// landed say so directly; older ones do not, and for those this falls back to
// the fingerprint such samples leave in the cost column.  A solve that gives up
// returns the same trajectory whatever sample it was asked about, so its cost
// repeats *bit for bit* across hundreds of rows -- which no genuinely solved
// sample ever does.  Finding that mode is general; the alternative, hard-coding
// the one cost value the cone demo happens to produce, is not.
vector<double> ReadNoOpColumn(const Sweep& sweep) {
  if (sweep.HasColumn("no_op_plan")) {
    return sweep.Column("no_op_plan");
  }
  vector<double> no_op(sweep.num_samples(), 0.0);
  if (!sweep.HasColumn("c3_cost")) {
    return no_op;
  }
  const vector<double>& cost = sweep.Column("c3_cost");
  std::map<double, int> counts;
  for (double value : cost) {
    if (std::isfinite(value)) ++counts[value];
  }
  double mode = 0.0;
  int mode_count = 0;
  for (const auto& [value, count] : counts) {
    if (count > mode_count) {
      mode = value;
      mode_count = count;
    }
  }
  // Two percent of the sweep landing on one exact double is far past
  // coincidence, and well under the ~17% such samples occupy in practice.
  if (mode_count < std::max(2, sweep.num_samples() / 50)) {
    return no_op;
  }
  for (int i = 0; i < sweep.num_samples(); ++i) {
    no_op[i] = cost[i] == mode ? 1.0 : 0.0;
  }
  std::cout << "This sweep predates the no_op_plan column; falling back to the "
            << mode_count << " samples sharing the repeated c3_cost " << mode
            << "." << std::endl;
  return no_op;
}

// The --slice_by/--slice_range window, as a per-sample mask.  Everything true
// when no slice was asked for.
vector<bool> ReadSlice(const Sweep& sweep) {
  vector<bool> in_slice(sweep.num_samples(), true);
  if (FLAGS_slice_by.empty()) {
    if (!FLAGS_slice_range.empty()) {
      throw std::runtime_error("--slice_range needs --slice_by");
    }
    return in_slice;
  }
  if (!sweep.HasColumn(FLAGS_slice_by)) {
    throw std::runtime_error("--slice_by names no column in this sweep: " +
                             FLAGS_slice_by);
  }
  double lo = 0.0;
  double hi = 0.0;
  if (std::sscanf(FLAGS_slice_range.c_str(), "%lf,%lf", &lo, &hi) != 2) {
    throw std::runtime_error("--slice_range must be 'lo,hi', got '" +
                             FLAGS_slice_range + "'");
  }
  const vector<double>& values = sweep.Column(FLAGS_slice_by);
  int num_in = 0;
  for (int i = 0; i < sweep.num_samples(); ++i) {
    in_slice[i] = values[i] >= lo && values[i] <= hi;
    num_in += in_slice[i] ? 1 : 0;
  }
  std::cout << "Slicing on " << FLAGS_slice_by << " in [" << lo << ", " << hi
            << "]: " << num_in << " of " << sweep.num_samples()
            << " samples drawn, and the color scales come from those alone."
            << std::endl;
  return in_slice;
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  Sweep sweep = ReadSweep(FLAGS_sweep_file);
  std::cout << "Read " << sweep.num_samples() << " samples from "
            << FLAGS_sweep_file << std::endl;
  AppendDerivedMetrics(&sweep);

  const string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  auto controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  auto vis_params = drake::yaml::LoadYamlFile<SamplingC3VisualizerParams>(
      controller_params.vis_params_file);

  // --- The scene: printer (and its ramp) plus the cone. ---
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Add3DPrinterToPlant(&plant, &scene_graph, /*include_ee=*/true);
  AddObjectsToPlant(&plant, &scene_graph, vis_params.object_vis_models);
  plant.Finalize();

  MeshcatVisualizerParams visualizer_params;
  visualizer_params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<Meshcat>();
  meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                          visualizer_params);

  // The goal's keep-out regions, drawn so they can be toggled against the
  // force field.  Same call the real visualizer makes.
  systems::StaticModelDrawer keep_out_drawer(meshcat);
  if (controller_params.keep_out_model_sequence.has_value()) {
    const auto& keep_out_models = *controller_params.keep_out_model_sequence;
    const int step = std::clamp(sweep.goal_step, 0,
                                static_cast<int>(keep_out_models.size()) - 1);
    if (!keep_out_models.empty() && !keep_out_models.at(step).empty()) {
      keep_out_drawer.AddModel(
          FindResourceOrThrow(keep_out_models.at(step)), "keep_out_base",
          "keep_out/goal_" + std::to_string(step), vis_params.keep_out_color);
    }
  }

  // The three C3 states the sweep solved against, under c3_state/ so they can
  // be toggled together or one at a time.  Blue-family colors throughout, to
  // stay clear of the samples' green-to-red ramp.
  const drake::geometry::Rgba kActualColor(0.20, 0.45, 1.00, 1.0);
  const drake::geometry::Rgba kTargetColor(0.00, 0.85, 0.95, 0.9);
  const drake::geometry::Rgba kFinalTargetColor(0.70, 0.30, 1.00, 1.0);
  bool drew_c3_states = false;
  if (sweep.x_actual.size() >= 10 && sweep.x_target.size() >= 10 &&
      sweep.x_final_target.size() >= 10) {
    DrawObjectState(meshcat.get(), "c3_state/actual_object", sweep.x_actual,
                    kActualColor);
    DrawObjectState(meshcat.get(), "c3_state/target_object", sweep.x_target,
                    kTargetColor);
    DrawObjectState(meshcat.get(), "c3_state/final_target_object",
                    sweep.x_final_target, kFinalTargetColor);
    // Only the target EEs are drawn.  The actual EE in the frozen state is a
    // placeholder that every sample overwrites -- the point cloud is the set of
    // actual EE positions, so drawing one here would be a lie.
    DrawEEState(meshcat.get(), "c3_state/target_ee", sweep.x_target,
                kTargetColor);
    DrawEEState(meshcat.get(), "c3_state/final_target_ee", sweep.x_final_target,
                kFinalTargetColor);
    drew_c3_states = true;
  } else {
    std::cout << "WARNING: this sweep file predates the c3_state_* header "
                 "lines, so the C3 states cannot be drawn."
              << std::endl;
  }

  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Park the printer clear of the cone and ramp.
  VectorXd printer_joints(3);
  {
    std::istringstream stream(FLAGS_printer_joints);
    string token;
    int i = 0;
    while (std::getline(stream, token, ',') && i < 3) {
      printer_joints(i++) = std::stod(token);
    }
    if (i != 3) {
      throw std::runtime_error("--printer_joints needs three values, got '" +
                               FLAGS_printer_joints + "'");
    }
  }
  const char* kJointNames[] = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};
  for (int i = 0; i < 3; ++i) {
    plant.GetJointByName<drake::multibody::PrismaticJoint>(kJointNames[i])
        .set_translation(&plant_context, printer_joints(i));
  }

  // Place the cone exactly where the sweep froze it.
  const Quaterniond object_quaternion(
      sweep.object_orientation(0), sweep.object_orientation(1),
      sweep.object_orientation(2), sweep.object_orientation(3));
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName(controller_params.base_names.at(0)),
      RigidTransformd(RotationMatrixd(object_quaternion.normalized()),
                      sweep.object_position));

  // --- The metric clouds. ---
  const vector<double>& ee_x = sweep.Column("ee_x");
  const vector<double>& ee_y = sweep.Column("ee_y");
  const vector<double>& ee_z = sweep.Column("ee_z");
  const vector<double> no_failures(sweep.num_samples(), 0.0);
  const vector<double>& fell_back = sweep.HasColumn("solve_fell_back")
                                        ? sweep.Column("solve_fell_back")
                                        : no_failures;
  if (!sweep.HasColumn("solve_fell_back")) {
    std::cout << "WARNING: this sweep file predates the solve_fell_back "
                 "column, so failed solves cannot be drawn black."
              << std::endl;
  }
  const vector<double> no_op = ReadNoOpColumn(sweep);
  const vector<bool> in_slice = ReadSlice(sweep);

  DrawnSamples drawn;
  for (int i = 0; i < sweep.num_samples(); ++i) {
    if (!in_slice[i]) continue;
    drawn.indices.push_back(i);
    const bool failed = fell_back[i] > 0.5;
    const bool did_nothing = no_op[i] > 0.5;
    drawn.is_real.push_back(!failed && !did_nothing);
    drawn.num_failed += failed ? 1 : 0;
    drawn.num_no_op += (!failed && did_nothing) ? 1 : 0;
  }
  if (drawn.indices.empty()) {
    throw std::runtime_error("--slice_by/--slice_range selected no samples");
  }

  // Every column past the sampler verdicts is something worth coloring by.
  vector<string> metrics;
  bool past_verdicts = false;
  for (const string& name : sweep.column_names) {
    if (name == "c3_cost") {
      past_verdicts = true;
    }
    if (past_verdicts) {
      metrics.push_back(name);
    }
  }

  const std::map<string, double> low_overrides =
      ParseOverrides(FLAGS_color_min);
  const std::map<string, double> high_overrides =
      ParseOverrides(FLAGS_color_max);
  const double point_size = FLAGS_point_size > 0.0
                                ? FLAGS_point_size
                                : vis_params.sample_buffer_point_size;

  std::map<string, ColorScale> scales;
  std::map<string, string> metric_paths;
  vector<string> drawn_metrics;
  vector<string> undeclared_metrics;
  for (const string& metric : metrics) {
    const vector<double>& values = sweep.Column(metric);

    // Percentiles come from the real samples only.  This is most of why the
    // cloud used to render as one flat color: the no-op samples sit near zero
    // force and are numerous enough to anchor the green end of the scale far
    // below anything the rest of the cloud reaches, compressing every real
    // sample into the top of the ramp.
    vector<double> solved;
    for (size_t k = 0; k < drawn.indices.size(); ++k) {
      const int i = drawn.indices[k];
      if (drawn.is_real[k] && std::isfinite(values[i])) {
        solved.push_back(values[i]);
      }
    }
    if (solved.empty()) {
      std::cout << "Skipping " << metric
                << ": no finite values from a successful solve." << std::endl;
      continue;
    }

    ColorScale scale;
    scale.num_solved = solved.size();
    scale.min = *std::min_element(solved.begin(), solved.end());
    scale.max = *std::max_element(solved.begin(), solved.end());
    scale.median = Percentile(solved, 50.0);
    scale.meaning = MeaningOf(metric);

    // Resolve the value range first, then decide which end of it is green.
    // Keeping the overrides as value bounds rather than color bounds means an
    // endpoint noted down for a metric stays meaningful whichever way that
    // metric points.
    double low = Percentile(solved, FLAGS_color_low_percentile);
    double high = Percentile(solved, FLAGS_color_high_percentile);
    if (auto it = low_overrides.find(metric); it != low_overrides.end()) {
      low = it->second;
    }
    if (auto it = high_overrides.find(metric); it != high_overrides.end()) {
      high = it->second;
    }
    if (scale.meaning == ColorMeaning::kJamRiskHighGreen) {
      scale.green = high;
      scale.red = low;
    } else {
      scale.green = low;
      scale.red = high;
    }

    const int num_drawn = static_cast<int>(drawn.indices.size());
    PointCloud cloud(num_drawn, drake::perception::pc_flags::kXYZs |
                                    drake::perception::pc_flags::kRGBs);
    Eigen::Matrix3Xf xyzs(3, num_drawn);
    Eigen::Matrix3Xi rgbs(3, num_drawn);
    const double span = scale.red - scale.green;
    for (int k = 0; k < num_drawn; ++k) {
      const int i = drawn.indices[k];
      xyzs.col(k) = Vector3d(ee_x[i], ee_y[i], ee_z[i]).cast<float>();
      if (fell_back[i] > 0.5 || !std::isfinite(values[i])) {
        // Black: the solve fell back to zero inputs, so this sample's force is
        // not a measurement and must not read as green.
        rgbs.col(k) = Eigen::Vector3i(0, 0, 0);
      } else if (!drawn.is_real[k]) {
        // Grey: the solve succeeded but produced no plan to execute, so the
        // low force it reports is the absence of a push, not a safe one.
        rgbs.col(k) = Eigen::Vector3i(128, 128, 128);
      } else {
        // t runs 0 at the green endpoint to 1 at the red one.  For a column
        // whose green end is the high one the span is negative, which flips
        // the ramp without needing a second branch.
        const double t = span != 0.0 ? (values[i] - scale.green) / span : 0.0;
        rgbs.col(k) = systems::RdYlGnReversedColor(t);
      }
    }
    cloud.mutable_xyzs() = xyzs;
    cloud.mutable_rgbs() = rgbs.cast<uint8_t>();

    metric_paths[metric] = MeshcatPath(metric, scale.meaning);
    meshcat->SetObject(metric_paths.at(metric), cloud, point_size);
    scales[metric] = scale;
    drawn_metrics.push_back(metric);
    if (scale.meaning == ColorMeaning::kUnrecognised) {
      undeclared_metrics.push_back(metric);
    }
  }

  if (drawn_metrics.empty()) {
    throw std::runtime_error("No metric had any usable values to draw");
  }
  // Start on the score if it could be built, and otherwise on a force metric
  // rather than the cost, since forces are the point.  Sweeps written before the
  // unretimed family was dropped name their retimed columns with a suffix, so
  // fall through to that before giving up and showing whatever came first.
  string shown_metric = drawn_metrics.front();
  for (const char* preferred :
       {"jam_risk_general", "max_u_norm_c3", "max_u_norm_c3_retimed"}) {
    if (std::find(drawn_metrics.begin(), drawn_metrics.end(), preferred) !=
        drawn_metrics.end()) {
      shown_metric = preferred;
      break;
    }
  }
  for (const string& metric : drawn_metrics) {
    if (metric != shown_metric) {
      meshcat->SetProperty(metric_paths.at(metric), "visible", false);
    }
  }

  // --- Report the scale, so the picture can be read in real units. ---
  const int kRuleWidth = 124;
  std::cout << "\n"
            << std::string(kRuleWidth, '-') << "\n"
            << std::left << std::setw(32) << "metric" << std::setw(16)
            << "green =" << std::right << std::setw(12) << "green"
            << std::setw(12) << "red" << "   |" << std::setw(12) << "min"
            << std::setw(12) << "median" << std::setw(12) << "max"
            << std::setw(9) << "solved" << "\n"
            << std::string(kRuleWidth, '-') << std::endl;
  std::cout << std::fixed;
  bool any_magnitude = false;
  for (const string& metric : drawn_metrics) {
    const ColorScale& scale = scales.at(metric);
    any_magnitude |= scale.meaning == ColorMeaning::kMagnitude ||
                     scale.meaning == ColorMeaning::kUnrecognised;
    std::cout << std::left << std::setw(32) << metric << std::setw(16)
              << GreenMeansLabel(scale.meaning) << std::right
              << std::setprecision(4) << std::setw(12) << scale.green
              << std::setw(12) << scale.red << "   |" << std::setw(12)
              << scale.min << std::setw(12) << scale.median << std::setw(12)
              << scale.max << std::setw(9) << scale.num_solved << std::endl;
  }
  std::cout << std::string(kRuleWidth, '-') << "\n"
            << drawn.indices.size() << " of " << sweep.num_samples()
            << " samples drawn; " << drawn.num_failed
            << " had a C3 solve fall back and are drawn BLACK, "
            << drawn.num_no_op
            << " produced no plan to execute and are drawn GREY (both "
               "excluded from every color scale).\n"
            << "Forces are in Newtons; max_delta_x_ee_* are in meters; "
               "c3_cost is unitless.  Values at or\npast the endpoints "
               "saturate to pure green / pure red.\n"
            << "\nWhat green means is stated per column above, and never "
               "assumed:\n"
               "  green = low jam risk   a jamming claim.  Usually the low end "
               "of the column, but the HIGH\n"
               "                         end for object travel and progress, "
               "since the object moving is\n"
               "                         what a jam prevents.\n"
               "  green = low cost       c3_cost alone.  An objective, not a "
               "risk: cheap is not safe.\n"
               "  green = low value      magnitude only, no claim either way.\n"
            << "\nEXPECT c3_cost AND THE RISK COLUMNS TO BE GREEN IN OPPOSITE "
               "PLACES.  That is the finding,\nnot a rendering bug: in this "
               "scene the cheapest decile of samples is 93% jammed."
            << std::endl;
  if (any_magnitude) {
    std::cout
        << "\nColumns labelled 'low value' carry no verdict -- a "
           "baseline, a property of the plan, or a\nquantity nobody has "
           "tied to an outcome.  They are drawn low-to-green so they stay "
           "readable;\ndo not read their color as good or bad."
        << std::endl;
  }
  if (scales.count("jam_risk_general") > 0) {
    std::cout << "\njam_risk_general is the three-term score: how far the plan "
                 "asks the end effector to\ngo, how far the rollout diverges "
                 "from it per meter asked, and the effort spent per\nmeter the "
                 "object's furthest material point moves.  No term names an "
                 "axis, and the\nlast one counts rotation as motion rather "
                 "than as risk, so the score carries no\nassumption about "
                 "which way the object should travel.  Toggle "
                 "ee_tracking_frac and\nlog_u_per_point_travel to see where "
                 "each term is doing the work; neither separates\njams on its "
                 "own, which is why the score is a column and not advice."
              << std::endl;
    // How well it lines up with the labels, if this file carries them -- the
    // number behind the picture the viewer is about to compare by eye.
    if (sweep.HasColumn("jammed")) {
      const vector<double>& risk = sweep.Column("jam_risk_general");
      const vector<double>& jammed = sweep.Column("jammed");
      vector<std::pair<double, double>> pairs;
      for (size_t k = 0; k < drawn.indices.size(); ++k) {
        const int i = drawn.indices[k];
        if (drawn.is_real[k] && std::isfinite(risk[i]) &&
            std::isfinite(jammed[i])) {
          pairs.emplace_back(risk[i], jammed[i]);
        }
      }
      if (pairs.size() >= 10) {
        std::sort(pairs.begin(), pairs.end());
        std::cout << "Jam rate by decile of the score, lowest risk first: ";
        std::cout << std::setprecision(0);
        for (int d = 0; d < 10; ++d) {
          const size_t lo = d * pairs.size() / 10;
          const size_t hi = (d + 1) * pairs.size() / 10;
          double jam_rate = 0.0;
          for (size_t k = lo; k < hi; ++k) {
            jam_rate += pairs[k].second;
          }
          std::cout << std::setw(4) << 100.0 * jam_rate / (hi - lo) << "%";
        }
        std::cout << std::setprecision(4) << "\nCOMPARING THIS AGAINST THE "
                     "jammed CLOUD IS ONLY PART OF A TEST: the score's weights\n"
                     "were fit on this sweep's labels.  Draw it over a sweep it "
                     "has not seen -- another\ngoal step, or another object "
                     "pose -- for the honest version."
                  << std::endl;
      }
    }
  }
  if (!undeclared_metrics.empty()) {
    std::cout << "\nNOTE: green has not been given a meaning for";
    for (const string& metric : undeclared_metrics) {
      std::cout << " " << metric;
    }
    std::cout << ".\n      Add them to MeaningOf() in jamming_visualizer.cc; "
                 "until then they are drawn as\n      'low value' and claim "
                 "nothing."
              << std::endl;
  }

  if (drew_c3_states) {
    std::cout << "\nC3 states, drawn as triads under c3_state/:\n"
              << "  actual_object       blue    object at "
              << sweep.x_actual.segment(7, 3).transpose() << "\n"
              << "  target_object       cyan    object goal at "
              << sweep.x_target.segment(7, 3).transpose() << "\n"
              << "  final_target_object purple  object goal at "
              << sweep.x_final_target.segment(7, 3).transpose() << "\n"
              << "  target_ee / final_target_ee (smaller triads) at "
              << sweep.x_target.head(3).transpose() << " / "
              << sweep.x_final_target.head(3).transpose() << "\n"
              << "The actual EE is not drawn: the frozen state's EE block is a "
                 "placeholder that\neach sample overwrites, so the point cloud "
                 "is the set of actual EE positions.\n"
              << "No lookahead sub-goal is applied offline, so the target and "
                 "final target\ncoincide unless --goal_step says otherwise."
              << std::endl;
  }

  // --- Publish once and hold the meshcat server open. ---
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  simulator.Initialize();
  simulator.AdvanceTo(0.0);
  diagram->ForcedPublish(simulator.get_context());

  std::cout << "\nMeshcat: " << meshcat->web_url() << "\n"
            << "Open Controls -> Scene -> jamming to toggle a metric on or "
               "off.  Each entry there\ncarries what green means for it, since "
               "that tree is the only text in the 3D view.\nShowing '"
            << metric_paths.at(shown_metric) << "' to start.\n"
            << "Ctrl-C to quit." << std::endl;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
