//
// Created by Sebastian Maierhofer on 08.11.20.
//

#include <geometry/curvilinear_coordinate_system.h>

#include "lane.h"


Lane::Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet lanelet, std::shared_ptr<CurvilinearCoordinateSystem> ccs)
    : Lanelet(lanelet), containedLanelets(std::move(containedLanelets)), curvilinearCoordinateSystem(std::move(ccs)) {}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const std::shared_ptr<CurvilinearCoordinateSystem> &Lane::getCurvilinearCoordinateSystem() const { return curvilinearCoordinateSystem; }
