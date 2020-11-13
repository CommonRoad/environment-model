//
// Created by Sebastian Maierhofer on 28.10.20.
//

#ifndef ENV_MODEL_STRUCTS_H
#define ENV_MODEL_STRUCTS_H


#include <iostream>
#include <vector>
#include "types.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


struct CycleElement{
    CycleElementType color;
    float duration;
};

// 2d vertices
struct vertice {
    double x;
    double y;
};


#endif //ENV_MODEL_STRUCTS_H
