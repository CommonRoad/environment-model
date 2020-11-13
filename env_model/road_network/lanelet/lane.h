//
// Created by Sebastian Maierhofer on 08.11.20.
//

#ifndef ENV_MODEL_LANE_H
#define ENV_MODEL_LANE_H

#include "lanelet.h"
#include "geometry/curvilinear_coordinate_system.h"

typedef geometry::CurvilinearCoordinateSystem CurvilinearCoordinateSystem;

class Lane {
public:
    Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet &lanelet, CurvilinearCoordinateSystem &ccs);

private:
        std::vector<std::shared_ptr<Lanelet>> containedLanelets;
        Lanelet lanelet;
        CurvilinearCoordinateSystem curvilinearCoordinateSystem;
};


#endif //ENV_MODEL_LANE_H
