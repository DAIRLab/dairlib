#include <igl/readOBJ.h>
#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

namespace bg = boost::geometry;
using BGPoint = bg::model::d2::point_xy<double>;
using BGPolygon = bg::model::polygon<BGPoint>;

int main(int argc, char** argv) {
    if (argc < 9) {
        std::cerr << "Usage: " << argv[0] << " <input_mesh.obj> <rot_x> <rot_y> <rot_z> <trans_x> <trans_y> <trans_z> <z_height>" << std::endl;
        return 1;
    }

    // Load mesh using libigl
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::readOBJ(argv[1], V, F)) {
        std::cerr << "Failed to load mesh from " << argv[1] << std::endl;
        return 1;
    }

    // Apply rotation and translation
    double rot_x = std::stod(argv[2]) * M_PI / 180.0;
    double rot_y = std::stod(argv[3]) * M_PI / 180.0;
    double rot_z = std::stod(argv[4]) * M_PI / 180.0;
    double trans_x = std::stod(argv[5]);
    double trans_y = std::stod(argv[6]);
    double trans_z = std::stod(argv[7]);
    double z_height = std::stod(argv[8]);

    Eigen::Matrix3d R = Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                        Eigen::AngleAxisd(rot_y, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                        Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d t(trans_x, trans_y, trans_z);

    for (int i = 0; i < V.rows(); ++i) {
        V.row(i) = (R * V.row(i).transpose()).transpose() + t.transpose();
    }

    // Find intersection points with the slicing plane
    std::vector<Eigen::Vector3d> intersections;
    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            Eigen::Vector3d p1 = V.row(F(i, j));
            Eigen::Vector3d p2 = V.row(F(i, (j + 1) % 3));
            if ((p1.z() - z_height) * (p2.z() - z_height) < 0) {
                double t = (z_height - p1.z()) / (p2.z() - p1.z());
                Eigen::Vector3d intersect = p1 + t * (p2 - p1);
                intersections.push_back(intersect);
            }
        }
    }

    if (intersections.empty()) {
        std::cerr << "No intersections found at z = " << z_height << std::endl;
        return 1;
    }

    // Convert intersections to 2D points
    std::vector<BGPoint> ring;
    for (const auto& pt : intersections) {
        ring.emplace_back(pt.x(), pt.y());
    }

    // Create polygon from ring
    BGPolygon poly;
    bg::assign_points(poly, ring);
    bg::correct(poly);

    // Buffer the polygon
    std::vector<BGPolygon> buffered_polygons;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(0.04);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy(5);
    bg::strategy::buffer::side_straight side_strategy;
    bg::buffer(poly, buffered_polygons, distance_strategy, side_strategy,
               join_strategy, end_strategy, point_strategy);

    if (buffered_polygons.empty()) {
        std::cerr << "Buffering resulted in no polygons." << std::endl;
        return 1;
    }

    // Sample points along the buffered polygon
    std::vector<BGPoint> sampled_points;
    const auto& outer = buffered_polygons.front().outer();
    double total_length = 0.0;
    for (size_t i = 0; i < outer.size() - 1; ++i) {
        total_length += bg::distance(outer[i], outer[i + 1]);
    }


    std::cout << "Number of points in the ring: " << ring.size() << std::endl;

    std::ofstream ring_out("examples/sampling_c3/sampling_generation/buffered_ring.csv");
    for (const auto& pt : outer) {
        ring_out << bg::get<0>(pt) << "," << bg::get<1>(pt) << std::endl;
    }
    ring_out.close();

    int num_samples = 100;
    double segment_length = total_length / num_samples;
    double accumulated_length = 0.0;
    size_t current_segment = 0;

    for (int i = 0; i < num_samples; ++i) {
        double target_length = i * segment_length;
        while (current_segment + 1 < outer.size() &&
               accumulated_length + bg::distance(outer[current_segment], outer[current_segment + 1]) < target_length) {
            accumulated_length += bg::distance(outer[current_segment], outer[current_segment + 1]);
            ++current_segment;
        }
        if (current_segment + 1 >= outer.size()) {
            break;
        }
        double remaining_length = target_length - accumulated_length;
        double seg_length = bg::distance(outer[current_segment], outer[current_segment + 1]);
        double ratio = seg_length == 0 ? 0 : remaining_length / seg_length;
        double x = bg::get<0>(outer[current_segment]) + ratio * (bg::get<0>(outer[current_segment + 1]) - bg::get<0>(outer[current_segment]));
        double y = bg::get<1>(outer[current_segment]) + ratio * (bg::get<1>(outer[current_segment + 1]) - bg::get<1>(outer[current_segment]));
        sampled_points.emplace_back(x, y);
    }

    // Write sampled points to CSV
    std::ofstream fout("examples/sampling_c3/sampling_generation/sampled_points.csv");
    for (const auto& pt : sampled_points) {
        fout << bg::get<0>(pt) << "," << bg::get<1>(pt) << std::endl;
    }
    fout.close();

    std::cout << "Sampled points written to 'sampled_points.csv'." << std::endl;
    return 0;
}
