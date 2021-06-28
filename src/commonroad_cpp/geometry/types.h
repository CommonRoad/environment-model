#pragma once

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type = boost::geometry::model::polygon<point_type>;
using box = boost::geometry::model::box<point_type>;
