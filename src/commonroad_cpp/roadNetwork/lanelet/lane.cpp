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
           std::shared_ptr<CurvilinearCoordinateSystem> ccs)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets), curvilinearCoordinateSystem(std::move(ccs)) {

    omp_init_lock(&ccs_lock);

    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets) {
    omp_init_lock(&ccs_lock);
    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const std::shared_ptr<CurvilinearCoordinateSystem> &Lane::getCurvilinearCoordinateSystem() {
    omp_set_lock(&ccs_lock);

    if (!curvilinearCoordinateSystem) {
        geometry::EigenPolyline temp_path;
        geometry::EigenPolyline reference_path;
        const auto &centerVertices = getCenterVertices();
        temp_path.reserve(centerVertices.size());
        for (auto vert : centerVertices)
            temp_path.push_back(Eigen::Vector2d(vert.x, vert.y));

        geometry::util::resample_polyline(temp_path, 2, reference_path);
        curvilinearCoordinateSystem = std::make_shared<CurvilinearCoordinateSystem>(reference_path);
    }

    omp_unset_lock(&ccs_lock);

    return curvilinearCoordinateSystem;
}

const std::unordered_set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }
