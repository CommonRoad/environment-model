//
// Created by sebastian on 08.11.20.
//

#include "lane.h"
#include <utility>
#include "boost/geometry.hpp"

namespace bg = boost::geometry;

Lane::Lane(std::vector<std::shared_ptr<Lanelet>>  containedLanelets, Lanelet lanelet, CurvilinearCoordinateSystem ccs) : containedLanelets(std::move(containedLanelets)),
                                                                                                                          lanelet(std::move(lanelet)), curvilinearCoordinateSystem(std::move(ccs)) {}

bool Lane::checkIntersection(const polygon_type &intersecting, size_t intersection_flag) const {
    return lanelet.checkIntersection(intersecting, intersection_flag);
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const {return containedLanelets;}

void Lane::setContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {containedLanelets = lanelets;}

Lanelet Lane::getLanelet() const {return lanelet;}

void Lane::setLanelet(const Lanelet &la) {lanelet = la;}

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() const {return curvilinearCoordinateSystem;}

void Lane::setCurvilinearCoordinateSystem(const CurvilinearCoordinateSystem &ccs) {curvilinearCoordinateSystem = ccs;}

