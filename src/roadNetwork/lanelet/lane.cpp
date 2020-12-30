//
// Created by Sebastian Maierhofer on 08.11.20.
//
#include "lane.h"

Lane::Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets,
           Lanelet lanelet,
           CurvilinearCoordinateSystem ccs) :
        containedLanelets(std::move(containedLanelets)),
        lanelet(std::move(lanelet)),
        curvilinearCoordinateSystem(std::move(ccs)) {}

Lanelet Lane::getLanelet() const { return lanelet; }

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() const { return curvilinearCoordinateSystem; }

bool Lane::checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const {
    return lanelet.checkIntersection(polygon_shape, intersection_type);
}

