//
// Created by sebastian on 28.10.20.
//

#ifndef ENV_MODEL_STRUCTS_H
#define ENV_MODEL_STRUCTS_H


#include <iostream>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> mpolygon_t;
typedef boost::geometry::model::linestring<point_type> linestring_t;
typedef boost::geometry::model::box<point_type> box;
using multi_linestring_type = boost::geometry::model::multi_linestring<linestring_t>;
typedef boost::geometry::model::segment<point_type> segment_t;

// 2d vertices
struct vertice {
    double x;
    double y;
};

// border of lanes
struct border {
    std::vector<vertice> vertices; // vertices of border
    std::vector<double> distances; // distance of each vertice to previous vertice
    std::vector<double> curvature; // curvature of the right border of each vertice
    size_t side;
};


#endif //ENV_MODEL_STRUCTS_H
