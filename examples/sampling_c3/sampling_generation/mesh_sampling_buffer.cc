#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/convex_hull_2.h>
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

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Kernel::Plane_3 Plane_3;
typedef std::vector<std::vector<Point_3>> Polylines;
typedef bg::model::d2::point_xy<double> BGPoint;
typedef bg::model::polygon<BGPoint> BGPolygon;
typedef Kernel::Point_2 Point_2;

BGPoint convert_to_bg_point(const Point_3& p) {
    return BGPoint(p.x(), p.y());
}

int main(int argc, char** argv) {
    // Argument parsing and mesh import
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_mesh_file>" << std::endl;
        return 1;
    }
    std::ifstream input_file(argv[1]);
    Mesh mesh;
    Mesh mesh2;
    if (!input_file) {
        std::cerr << "Cannot open file: " << argv[1] << std::endl;
        return 1;
    } else {
        std::cout << "File opened successfully: " << argv[1] << std::endl;
    }
    const std::string obj_path = argv[1];
    PMP::IO::read_polygon_mesh(obj_path, mesh);
    // CGAL::IO::read_OBJ(input_file, mesh);
    // CGAL::IO::write_OBJ("examples/sampling_c3/sampling_generation/cpp.obj", mesh);
    std::cout << "# Vertices: " << mesh.number_of_vertices() << std::endl;
    std::cout << "# Faces: " << mesh.number_of_faces() << std::endl;
    std::cout << "# Edges: " << mesh.number_of_edges() << std::endl;

    // Applying command line pose vector to the mesh
    double rot_x = std::stod(argv[2]) * M_PI / 180.0;
    double rot_y = std::stod(argv[3]) * M_PI / 180.0;
    double rot_z = std::stod(argv[4]) * M_PI / 180.0;
    double trans_x = std::stod(argv[5]);
    double trans_y = std::stod(argv[6]);
    double trans_z = std::stod(argv[7]);
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
    
    //Generating plane and slicing mesh
    double z_height = std::stod(argv[8]); 
    Plane_3 plane(0, 0, 1, -z_height);

    CGAL::Polygon_mesh_slicer<Mesh, Kernel> slicer(mesh);

    std::vector<Point_3> intersections;

    double z_min = 1e10, z_max = -1e10;
    for (auto v : mesh.vertices()) {
        double z = mesh.point(v).z();
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }

    for (auto e : mesh.edges()) {
        auto he = mesh.halfedge(e);
        auto v1 = mesh.source(he);
        auto v2 = mesh.target(he);

        Point_3 p1 = mesh.point(v1);
        Point_3 p2 = mesh.point(v2);

        double z1 = p1.z();
        double z2 = p2.z();

        if ((z1 - z_height) * (z2 - z_height) < 0) {
            double t = (z_height - z1) / (z2 - z1);
            double x = p1.x() + t * (p2.x() - p1.x());
            double y = p1.y() + t * (p2.y() - p1.y());
            intersections.emplace_back(x, y, z_height);
        }
    }

    std::cout << "Found " << intersections.size() << " intersection points on the slice plane." << std::endl;

    std::cout << "Min Z: " << z_min << std::endl;
    std::cout << "Max Z: " << z_max << std::endl;
    Polylines polylines;
    slicer(plane, std::back_inserter(polylines));

    // for (size_t i = 0; i < polylines.size(); ++i) {
    //     std::cout << "Polyline " << i << ":" << std::endl;
    //     for (size_t j = 0; j < polylines[i].size(); ++j) {
    //         const auto& pt = polylines[i][j];
    //         std::cout << "  Point " << j << ": (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    //     }
    // }

    if (polylines.empty()) {
        std::cerr << "Nothing on slice plane." << std::endl;
        return 1;
    }

    // Creating polygons from slice polylines
    std::vector<BGPolygon> bg_polys;
    for (const auto& polyline : polylines) {
        BGPolygon bg_poly;
        for (const auto& pt : polyline)
            bg::append(bg_poly.outer(), convert_to_bg_point(pt));
        if (polyline.front() != polyline.back())
            bg::append(bg_poly.outer(), convert_to_bg_point(polyline.front()));
        bg::correct(bg_poly);
        bg_polys.push_back(bg_poly);
    }

    std::cout << "Number of polygons created: " << bg_polys.size() << std::endl;
    if (bg_polys.empty()) {
        std::cerr << "No valid polygons created from the mesh slice." << std::endl;
        return 1;
    }

    // Unionizing polygons
    std::vector<BGPolygon> union_result = bg_polys;

    if (bg_polys.empty()) {
        return 1;
    }
    if (bg_polys.size() == 1) {
        union_result = bg_polys;
    } else {
        std::vector<BGPolygon> temp_result;
        bg::union_(union_result[0], union_result[1], temp_result);
        for (size_t i = 2; i < union_result.size(); ++i) {
            std::vector<BGPolygon> temp2;
            for (const auto& poly : temp_result) {
                bg::union_(poly, union_result[i], temp2);
            }
            temp_result = std::move(temp2);
        }
    }

    std::cout << "Number of unioned polygons: " << union_result.size() << std::endl;

    std::ofstream unioned_out("examples/sampling_c3/sampling_generation/unioned_points.csv");
    for (const auto& poly : union_result) {
        for (const auto& pt : poly.outer()) {
            unioned_out << bg::get<0>(pt) << "," << bg::get<1>(pt) << std::endl;
        }
        unioned_out << std::endl;
    }
    unioned_out.close();
    std::cout << "Unioned polygon points written to 'unioned_points.csv'." << std::endl;

    // Buffering the unioned polygons
    std::vector<BGPolygon> buffered_polygons;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(0.04);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy(5);
    bg::strategy::buffer::side_straight side_strategy;
    for (const auto& poly : union_result) {
        std::vector<BGPolygon> temp;
        bg::buffer(poly, temp, distance_strategy, side_strategy,
                   join_strategy, end_strategy, point_strategy);
        buffered_polygons.insert(buffered_polygons.end(), temp.begin(), temp.end());
    }

    std::cout << "Made it to the sampling step." << std::endl;
    std::cout << "Number of buffered polygons: " << buffered_polygons.size() << std::endl;

    // Making outer rings from the buffered polygons
    std::vector<BGPoint> ring;
    // for (const auto& poly : buffered_polygons) {
    //     const auto& outer = poly.outer();
    //     ring.insert(ring.end(), outer.begin(), outer.end());
    // }

    // Use only the outer ring of the first buffered polygon
    if (!buffered_polygons.empty()) {
        ring = buffered_polygons[0].outer();
    }

    std::cout << "Number of points in the ring: " << ring.size() << std::endl;

    std::ofstream ring_out("examples/sampling_c3/sampling_generation/buffered_ring.csv");
    for (const auto& pt : ring) {
        ring_out << bg::get<0>(pt) << "," << bg::get<1>(pt) << std::endl;
    }
    ring_out.close();

    // Trying convex hull on the outer ring
    std::vector<Point_2> convex_hull;
    std::vector<Point_2> convex_hull_points;
    for (const auto& bgpt : ring) {
        convex_hull_points.push_back(Point_2(bg::get<0>(bgpt), bg::get<1>(bgpt)));
    }
    CGAL::convex_hull_2(convex_hull_points.begin(), convex_hull_points.end(), std::back_inserter(convex_hull));
    std::cout << "Convex hull size: " << convex_hull.size() << std::endl;
    std::ofstream hull_out("examples/sampling_c3/sampling_generation/convex_hull.csv");
    for (const auto& pt : convex_hull) {
        hull_out << pt.x() << "," << pt.y() << std::endl;
    }
    hull_out.close();


    // Sampling points along the generated outer ring
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
