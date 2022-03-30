//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lane.h"
#include "../../geometry/geometric_operations.h"
#include <geometry/curvilinear_coordinate_system.h>
#include <utility>

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet,
           CurvilinearCoordinateSystem ccs)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets), curvilinearCoordinateSystem(std::move(ccs)) {
    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets) {
    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const CurvilinearCoordinateSystem &Lane::getCurvilinearCoordinateSystem() {
    if (!curvilinearCoordinateSystem) {
        geometry::EigenPolyline reference_path;
        for (auto vert : getCenterVertices())
            reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));

        geometry::util::resample_polyline(reference_path, 2, reference_path);
        curvilinearCoordinateSystem = CurvilinearCoordinateSystem(reference_path);
    }
    return curvilinearCoordinateSystem.value();
}

const std::set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }

std::vector<std::shared_ptr<Lanelet>> Lane::getSuccessorLanelets(const std::shared_ptr<Lanelet> &lanelet) const {
    if (getContainedLaneletIDs().find(lanelet->getId()) == getContainedLaneletIDs().end())
        return {};
    std::vector<std::shared_ptr<Lanelet>> relevantLanelets;
    auto succs{lanelet->getSuccessors()};
    while (!succs.empty())
        for (const auto &suc : succs) {
            if (getContainedLaneletIDs().find(suc->getId()) != getContainedLaneletIDs().end()) {
                relevantLanelets.push_back(suc);
                succs = relevantLanelets.back()->getSuccessors();
                break;
            }
        }

    return relevantLanelets;
}
