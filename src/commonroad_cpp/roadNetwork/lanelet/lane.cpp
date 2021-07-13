//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lane.h"

#include "../../geometry/geometric_operations.h"
#include <utility>

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet,
           CurvilinearCoordinateSystem ccs)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets), curvilinearCoordinateSystem(std::move(ccs)) {
    for (const auto &la : containedLanelets)
        containedLaneletIds.insert(la->getId());
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() const { return curvilinearCoordinateSystem; }

const std::set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }
