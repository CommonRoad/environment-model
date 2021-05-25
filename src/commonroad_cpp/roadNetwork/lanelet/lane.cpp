//
// Created by Sebastian Maierhofer on 08.11.20.
//
#include "lane.h"

Lane::Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet lanelet, CurvilinearCoordinateSystem ccs)
    : Lanelet(lanelet), containedLanelets(std::move(containedLanelets)), curvilinearCoordinateSystem(std::move(ccs)) {}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() const { return curvilinearCoordinateSystem; }
