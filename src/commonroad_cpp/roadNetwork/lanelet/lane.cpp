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

/* NOTE: The Lane destructor is explicitly instantiated here so that the deallocation calls are hopefully
 * compiled with the same settings as the allocation calls.
 * This is critical since the CCS uses Eigen internally which uses a custom allocator to ensure appropriate
 * alignment of certain structures. If different allocators are used for the allocation and deallocation,
 * silent memory corruption will occur resulting in hard to find bugs.
 */
Lane::~Lane() {}

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

        geometry::util::resample_polyline(temp_path, 2, reference_path); // todo parameter
        curvilinearCoordinateSystem = std::make_shared<CurvilinearCoordinateSystem>(reference_path);
    }

    omp_unset_lock(&ccs_lock);

    return curvilinearCoordinateSystem;
}

const std::unordered_set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }

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
