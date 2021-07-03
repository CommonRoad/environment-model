//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lane.h"
#include "../../geometry/geometric_operations.h"

Lane::Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet lanelet, CurvilinearCoordinateSystem ccs)
    : Lanelet(lanelet), containedLanelets(std::move(containedLanelets)), curvilinearCoordinateSystem(std::move(ccs)),
      orientation(geometric_operations::computeOrientationFromPolyline(lanelet.getCenterVertices())),
      pathLength(geometric_operations::computePathLengthFromPolyline(lanelet.getCenterVertices())){
    for (const auto &la : containedLanelets)
        containedLaneletIds.insert(la->getId());
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() const { return curvilinearCoordinateSystem; }

std::set<size_t> Lane::getContainedLaneletIDs() {
    std::set<size_t> idSet;
    for (const auto &la : containedLanelets)
        idSet.insert(la->getId());
    return idSet;
}
