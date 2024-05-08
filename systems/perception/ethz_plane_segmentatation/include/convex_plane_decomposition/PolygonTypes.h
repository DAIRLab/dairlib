//
// Created by rgrandia on 07.06.20.
//

#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

namespace convex_plane_decomposition {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint2d = K::Point_2;
using CgalCircle2d = K::Circle_2;
using CgalPolygon2d = CGAL::Polygon_2<K>;
using CgalSegment2d = CgalPolygon2d::Segment_2;
using CgalPolygonWithHoles2d = CGAL::Polygon_with_holes_2<K>;
namespace CgalPolylineSimplification = CGAL::Polyline_simplification_2;
using CgalSquaredDistanceCost = CgalPolylineSimplification::Squared_distance_cost;
using CgalStopBelowCountThreshold = CgalPolylineSimplification::Stop_below_count_threshold;
using CgalStopBelowCountRationThreshold = CgalPolylineSimplification::Stop_below_count_ratio_threshold;
using CgalBbox2d = CGAL::Bbox_2;

}  // namespace convex_plane_decomposition
