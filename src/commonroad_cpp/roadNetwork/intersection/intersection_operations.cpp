#include <algorithm>
#include <unordered_set>

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

bool intersection_operations::onIncoming(const size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    const auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lanelets)
        if (let->hasLaneletType(LaneletType::incoming))
            return true;

    return false;
}

bool intersection_operations::checkSameIncoming(const std::shared_ptr<Lanelet> &letK,
                                                const std::shared_ptr<Lanelet> &letP, const double fov,
                                                const int numIntersections) {
    const auto simLaneletsK{lane_operations::combineLaneLanelets(
        lane_operations::combineLaneletAndPredecessorsToLane(letK, fov, numIntersections))};
    const auto simLaneletsP{lane_operations::combineLaneLanelets(
        lane_operations::combineLaneletAndPredecessorsToLane(letP, fov, numIntersections))};

    std::unordered_set<size_t> simPIds;
    simPIds.reserve(simLaneletsP.size());
    for (const auto &l : simLaneletsP)
        simPIds.insert(l->getId());

    for (const auto &laK : simLaneletsK) {
        if (!laK->hasLaneletType(LaneletType::incoming))
            continue;
        for (const auto &adjLet : lanelet_operations::adjacentLanelets(laK)) {
            if (simPIds.count(adjLet->getId()))
                return true;
        }
    }
    return false;
}

void intersection_operations::findLeftOf(const std::shared_ptr<IncomingGroup> &origin,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!origin->getRightOutgoings().empty()) {
        if (const auto out = roadNetwork->findOutgoingGroupByLanelet(origin->getRightOutgoings()[0]))
            origin->setIsLeftOf(roadNetwork->findIncomingGroupByOutgoingGroup(out));
    }
}

std::vector<std::shared_ptr<Intersection>>
intersection_operations::currentIntersection(const size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK) {
    // Use lanelet→intersection index for O(occupied_lanelets) instead of
    // O(intersections × member_lanelets × occupied_lanelets).
    std::vector<std::shared_ptr<Intersection>> result;
    std::unordered_set<size_t> seen;
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &let : lanelets) {
        for (const auto &inter :
             world->getRoadNetwork()->findIntersectionsByLaneletId(let->getId(), world->getRoadNetwork())) {
            if (seen.insert(inter->getId()).second)
                result.push_back(inter);
        }
    }
    return result;
}

std::shared_ptr<IncomingGroup> intersection_operations::currentIncoming(const size_t timeStep,
                                                                        const std::shared_ptr<World> &world,
                                                                        const std::shared_ptr<Obstacle> &obs) {
    // Use lanelet→incoming-group index for O(occupied_lanelets) instead of
    // O(intersections × incoming_groups × incoming_lanelets × occupied_lanelets).
    const auto lanelets = obs->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &let : lanelets) {
        if (const auto incoming = world->getRoadNetwork()->findIncomingGroupByLanelet(let))
            return incoming;
    }
    return nullptr;
}

IntersectionType intersection_operations::matchStringToIntersectionType(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), toupper);
    if (str == "T_INTERSECTION")
        return IntersectionType::T_INTERSECTION;
    if (str == "FOUR_WAY_STOP_INTERSECTION")
        return IntersectionType::FOUR_WAY_STOP_INTERSECTION;
    if (str == "FOUR_WAY_INTERSECTION")
        return IntersectionType::FOUR_WAY_INTERSECTION;
    if (str == "UNCONTROLLED_INTERSECTION")
        return IntersectionType::UNCONTROLLED_INTERSECTION;
    return IntersectionType::UNKNOWN;
}
