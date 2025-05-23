#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/IO/OBJ.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstdlib>

namespace bg = boost::geometry;
namespace PMP = CGAL::Polygon_mesh_processing;

// typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Kernel::Plane_3 Plane_3;
typedef std::vector<std::vector<Point_3>> Polylines;
typedef bg::model::d2::point_xy<double> BGPoint;
typedef bg::model::polygon<BGPoint> BGPolygon;

BGPoint convert_to_bg_point(const Point_3& p) {
    return BGPoint(p.x(), p.y());
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_mesh_file>" << std::endl;
        return 1;
    }
    std::ifstream input_file(argv[1]);
    Mesh mesh;
    if (!input_file) {
        std::cerr << "Cannot open file: " << argv[1] << std::endl;
        return 1;
    } else {
        std::cout << "File opened successfully: " << argv[1] << std::endl;
    }
    CGAL::IO::read_OBJ(input_file, mesh);

    std::cout << "# Vertices: " << mesh.number_of_vertices() << std::endl;
    
    double rot_x = std::stod(argv[2]) * M_PI / 180.0;
    double rot_y = std::stod(argv[3]) * M_PI / 180.0;
    double rot_z = std::stod(argv[4]) * M_PI / 180.0;
    double trans_x = std::stod(argv[5]);
    double trans_y = std::stod(argv[6]);
    double trans_z = std::stod(argv[7]);
    //transform mesh
    Eigen::Matrix3d R = Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ()).toRotationMatrix() * 
                        Eigen::AngleAxisd(rot_y, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                        Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d t(trans_x, trans_y, trans_z);
    for (auto v : mesh.vertices()) {
        Point_3 p = mesh.point(v);
        Eigen::Vector3d p_eigen(p.x(), p.y(), p.z());
        Eigen::Vector3d transformed_point = R * p_eigen + t;
        mesh.point(v) = Point_3(transformed_point.x(), transformed_point.y(), transformed_point.z());
    }

    
    double z_height = 0.005; 
    Plane_3 plane(0, 0, 1, -z_height);
    
    CGAL::Polygon_mesh_slicer<Mesh, Kernel> slicer(mesh);

    double z_min = 1e10, z_max = -1e10;
    for (auto v : mesh.vertices()) {
        double z = mesh.point(v).z();
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }
    std::cout << "Min Z: " << z_min << std::endl;
    std::cout << "Max Z: " << z_max << std::endl;
    Polylines polylines;
    slicer(plane, std::back_inserter(polylines));

    if (polylines.empty()) {
        std::cerr << "Nothing on slice plane." << std::endl;
        return 1;
    }

    const auto& polygon = polylines[0];
    BGPolygon bg_poly;
    for (const auto& pt : polygon)
        bg::append(bg_poly.outer(), convert_to_bg_point(pt));
    if (polygon.front() != polygon.back())
        bg::append(bg_poly.outer(), convert_to_bg_point(polygon.front()));
    

    std::cout << "Made it to the buffer step." << std::endl;
    std::cout << "Number of points in the polygon: " << bg_poly.outer().size() << std::endl;
    std::cout << "First point: " << bg::get<0>(bg_poly.outer()[0]) << ", " << bg::get<1>(bg_poly.outer()[0]) << std::endl;
    std::cout << "Last point: " << bg::get<0>(bg_poly.outer()[bg_poly.outer().size() - 1]) << ", " << bg::get<1>(bg_poly.outer()[bg_poly.outer().size() - 1]) << std::endl;
    bg::validity_failure_type failure;
    if(!bg::is_valid(bg_poly, failure)) {
        std::cerr << "Polygon is not valid. Failure code: " << failure << std::endl;
    }
    bg::correct(bg_poly);
    if(!bg::is_valid(bg_poly, failure)) {
        std::cerr << "Polygon is still not valid after correction. Failure code: " << failure << std::endl;
    }

    //buffering
    std::vector<BGPolygon> buffered_polygons;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(0.03);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy(10);
    bg::strategy::buffer::side_straight side_strategy;
    bg::buffer(bg_poly, buffered_polygons, distance_strategy, side_strategy,
                join_strategy, end_strategy, point_strategy);
    

    std::cout << "Made it to the sampling step." << std::endl;
    std::cout << "Number of buffered polygons: " << buffered_polygons.size() << std::endl;


    //sampling
    const auto& ring = buffered_polygons[0].outer();


    std::cout << "Number of points in the ring: " << ring.size() << std::endl;


    double total_length = 0;
    for (size_t i = 0; i < ring.size() - 1; ++i) {
        total_length += bg::distance(ring[i], ring[i + 1]);
    }
    int num_samples = 100;
    double segment_length = total_length / num_samples;
    std::vector<BGPoint> sampled_points;

    size_t current_segment = 0;
    double sum_length = 0;


    std::cout << "Total length: " << total_length << std::endl;


    for (int i = 0; i < num_samples; ++i) {
        double target_length = i * segment_length;
        while (current_segment + 1 < ring.size() &&
               sum_length + bg::distance(ring[current_segment], ring[current_segment + 1]) < target_length) 
               {
            sum_length += bg::distance(ring[current_segment], ring[current_segment + 1]);
            ++current_segment;
        }
        if (current_segment + 1 >= ring.size()) {
            break;
        }
        double remaining_length = target_length - sum_length;
        double seg = bg::distance(ring[current_segment], ring[current_segment + 1]);
        double ratio = seg == 0 ? 0 : remaining_length / seg;
        double x = bg::get<0>(ring[current_segment]) + ratio * (bg::get<0>(ring[current_segment + 1]) - bg::get<0>(ring[current_segment]));
        double y = bg::get<1>(ring[current_segment]) + ratio * (bg::get<1>(ring[current_segment + 1]) - bg::get<1>(ring[current_segment]));
        sampled_points.emplace_back(x, y);
    }
    std::ofstream fout("examples/sampling_c3/sampling_generation/sampled_points.csv");
    for (const auto& pt : sampled_points) {
        fout << boost::geometry::get<0>(pt) << "," << boost::geometry::get<1>(pt) << std::endl;
    }
    fout.close();

    std::cout << "Sampled points written to 'sampled_points.csv'. Plot with Python or another tool!" << std::endl;
}
