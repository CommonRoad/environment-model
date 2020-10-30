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

struct ShortestPath {
    std::vector<size_t> indexBorder;
    std::vector<double> xi;
    std::vector<size_t> side;
    std::vector<double> curvature;
};

/*
 * time horizon for prediction
 */
struct timeStruct {
    float startingTime;
    float timeStep;
    float ending;
};

typedef struct {
    size_t begin;
    size_t end;
} region;

/*
 * configurable parameters by rqt_reconfigure
 * start rosrun rqt_reconfigure rqt_reconfigure to adjust them
 */
struct obstacleParameters {
    double scale;
    double a_max_vehicles;
    double a_long_max_vehicles;
    double a_long_max_bicycle;
    double a_long_min_vehicles;
    double a_long_min_bicycle;
    double a_max_bicycle;
    double a_max_pedestrians;
    double a_stop_pedestrians;
    double v_max_vehicles;
    double v_s_vehicles;
    double v_s_bicycle;
    double v_max_pedestrians;
    double v_max_bicycle;
    double dSlack;
    double speedingFactor;
    size_t num_vertices;
    bool occRuleBased_pedestrians;
    bool occDynamicBased_pedestrians;
    bool occM1_vehicles;
    bool occM2_vehicles;
    bool occM3_vehicles;
    bool bstop;
    bool bslack;
    bool bperp;
    timeStruct timeHorizon;
};

/*
 * trajectory
 */
struct traj {
    std::vector<double> distances;
    std::vector<double> velocities;
    double timeStamp;
    int finalBound = -1;
    vertice final = {0.0, 0.0};
};

/*
 * trajectory struct for a lane
 */
struct trajLane {
    bool bounded = false;
    std::vector<traj> trajectory;
    size_t iPath;
    double projectedDist; // projected distance to closest center vertice
};

#endif //ENV_MODEL_STRUCTS_H
