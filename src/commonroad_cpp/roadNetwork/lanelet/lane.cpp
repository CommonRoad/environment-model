//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <utility>

#include <spdlog/spdlog.h>

/* NOTE: The Lane destructor is explicitly instantiated here so that the deallocation calls are hopefully
 * compiled with the same settings as the allocation calls.
 * This is critical since the CCS uses Eigen internally which uses a custom allocator to ensure appropriate
 * alignment of certain structures. If different allocators are used for the allocation and deallocation,
 * silent memory corruption will occur resulting in hard to find bugs.
 */
Lane::~Lane() { omp_destroy_lock(&ccs_lock); }

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
            temp_path.emplace_back(vert.x, vert.y);

        SPDLOG_DEBUG("Reference Path - initial size: {}", temp_path.size());
        geometry::util::chaikins_corner_cutting(temp_path, RoadNetworkParameters::cornerCuttingRefinements,
                                                reference_path);
        SPDLOG_DEBUG("Reference Path - after chaikins_corner_cutting: {} (refinements: {})", reference_path.size(),
                     refinements);

        geometry::util::resample_polyline(reference_path, RoadNetworkParameters::stepsToResamplePolyline, temp_path);

        SPDLOG_DEBUG("Reference Path - after resampling: {} (step size: {})", temp_path.size(), polyline_step_size);

        reference_path = temp_path;

        curvilinearCoordinateSystem =
            std::make_shared<CurvilinearCoordinateSystem>(reference_path, RoadNetworkParameters::projectionDomainLimit,
                                                          RoadNetworkParameters::eps1, RoadNetworkParameters::eps2);
    }

    omp_unset_lock(&ccs_lock);

    return curvilinearCoordinateSystem;
}

const std::unordered_set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }

bool Lane::containsLanelet(const std::shared_ptr<Lanelet> &lanelet) const { return containsLanelet(lanelet->getId()); }

bool Lane::containsLanelet(size_t laneletId) const {
    return containedLaneletIds.find(laneletId) != containedLaneletIds.cend();
}

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

bool Lane::contains(std::vector<std::shared_ptr<Lanelet>> lanelets) {
    auto containedLaneletIdsTmp{containedLaneletIds};
    return std::any_of(lanelets.begin(), lanelets.end(), [containedLaneletIdsTmp](const std::shared_ptr<Lanelet> &let) {
        return containedLaneletIdsTmp.find(let->getId()) != containedLaneletIdsTmp.end();
    });
}