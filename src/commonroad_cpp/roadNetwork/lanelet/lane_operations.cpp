#include <algorithm>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>

#include <range/v3/all.hpp>

std::vector<std::vector<std::shared_ptr<Lanelet>>> lane_operations::combineLaneletAndSuccessorsToLane(
    const std::shared_ptr<Lanelet> &curLanelet, double fov, int numIntersections,
    const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, double offset) {
    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{offset}; // neglect initial lanelet
    for (const auto &lanelet : laneletList)
        laneLength += lanelet->getPathLength().back();
    if (curLanelet->hasLaneletType(LaneletType::incoming))
        --numIntersections;

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getSuccessors().empty() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &lanelet) {
                         return curLanelet->getId() == lanelet->getId();
                     }) and
        laneLength < fov and numIntersections >= 0) {
        for (const auto &lanelet : curLanelet->getSuccessors()) {
            auto newLanes{combineLaneletAndSuccessorsToLane(lanelet, fov, numIntersections, laneletList, offset)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>> lane_operations::combineLaneletAndPredecessorsToLane(
    const std::shared_ptr<Lanelet> &curLanelet, double fov, int numIntersections,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets, double offset) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{offset}; // neglect initial lanelet
    for (const auto &lanelet : laneletList)
        laneLength += lanelet->getPathLength().back();
    if (curLanelet->hasLaneletType(LaneletType::incoming))
        --numIntersections;

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getPredecessors().empty() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &lanelet) {
                         return curLanelet->getId() == lanelet->getId();
                     }) and
        laneLength < fov and numIntersections >= 0) {
        for (const auto &lanelet : curLanelet->getPredecessors()) {
            auto newLanes{combineLaneletAndPredecessorsToLane(lanelet, fov, numIntersections, laneletList, offset)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lane_operations::createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                                             const std::shared_ptr<RoadNetwork> &roadNetwork, double fovRear,
                                             double fovFront, int numIntersections, vertex position) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &lanelet : initialLanelets) {
        // lane was already created based on this initial lanelet -> continue with next lanelet
        auto newLanes{roadNetwork->findLanesByBaseLanelet(lanelet->getId())};
        if (!newLanes.empty()) {
            for (const auto &newLane : newLanes) {
                auto idx{newLane->findClosestIndex(position.x, position.y, true)};
                double rearLength{newLane->getPathLength().at(idx)};
                double frontLength{newLane->getPathLength().back() - newLane->getPathLength().at(idx)};
                if (!std::any_of(lanes.begin(), lanes.end(),
                                 [newLane](const std::shared_ptr<Lane> &lane) {
                                     return newLane->getContainedLaneletIDs() == lane->getContainedLaneletIDs();
                                 }) and
                    (frontLength > fovFront or newLane->getContainedLanelets().back()->getSuccessors().empty()) and
                    (rearLength > fovRear or newLane->getContainedLanelets().at(0)->getPredecessors().empty()))
                    lanes.push_back(newLane);
            }
            if (!lanes.empty())
                continue;
        }

        auto idx{lanelet->findClosestIndex(position.x, position.y, true)};
        auto newLaneSuccessorParts{combineLaneletAndSuccessorsToLane(lanelet, fovFront, numIntersections, {},
                                                                     -lanelet->getPathLength().at(idx))};
        auto newLanePredecessorParts{
            combineLaneletAndPredecessorsToLane(lanelet, fovRear, numIntersections, {},
                                                lanelet->getPathLength().at(idx) - lanelet->getPathLength().back())};
        if (!newLaneSuccessorParts.empty() and !newLanePredecessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                for (const auto &lanePre : newLanePredecessorParts) {
                    std::vector<std::shared_ptr<Lanelet>> containedLanelets{lanePre};
                    std::reverse(containedLanelets.begin(), containedLanelets.end());
                    containedLanelets.insert(containedLanelets.end(), laneSuc.begin() + 1, laneSuc.end());
                    newLanes.push_back(
                        createLaneByContainedLanelets(containedLanelets, ++*roadNetwork->getIdCounterRef()));
                }
        else if (!newLaneSuccessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts) {
                newLanes.push_back(createLaneByContainedLanelets(laneSuc, ++*roadNetwork->getIdCounterRef()));
            }
        else
            for (const auto &lanePre : newLanePredecessorParts) {
                newLanes.push_back(createLaneByContainedLanelets(lanePre, ++*roadNetwork->getIdCounterRef()));
            }
        newLanes = roadNetwork->addLanes(newLanes, lanelet->getId());
        for (const auto &newLane : newLanes)
            if (!std::any_of(lanes.begin(), lanes.end(), [newLane](const std::shared_ptr<Lane> &lane) {
                    return newLane->getContainedLaneletIDs() == lane->getContainedLaneletIDs();
                }))
                lanes.push_back(newLane);
    }
    return lanes;
}

std::shared_ptr<Lane>
lane_operations::createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
                                               size_t newId) {
    std::set<ObstacleType> userOneWay;
    std::set<ObstacleType> userBidirectional;
    std::vector<vertex> centerVertices;
    std::vector<vertex> leftVertices;
    std::vector<vertex> rightVertices;
    std::set<LaneletType> typeList;
    long shift{0};

    for (const auto &lanelet : containedLanelets) {
        std::set_intersection(userOneWay.begin(), userOneWay.end(), lanelet->getUsersOneWay().begin(),
                              lanelet->getUsersOneWay().end(), std::inserter(userOneWay, userOneWay.begin()));

        std::set_intersection(userBidirectional.begin(), userBidirectional.end(),
                              lanelet->getUsersBidirectional().begin(), lanelet->getUsersBidirectional().end(),
                              std::inserter(userBidirectional, userBidirectional.begin()));

        centerVertices.insert(centerVertices.end(), lanelet->getCenterVertices().begin() + shift,
                              lanelet->getCenterVertices().end());

        leftVertices.insert(leftVertices.end(), lanelet->getLeftBorderVertices().begin() + shift,
                            lanelet->getLeftBorderVertices().end());

        rightVertices.insert(rightVertices.end(), lanelet->getRightBorderVertices().begin() + shift,
                             lanelet->getRightBorderVertices().end());

        std::set_intersection(typeList.begin(), typeList.end(), lanelet->getLaneletTypes().begin(),
                              lanelet->getLaneletTypes().end(), std::inserter(typeList, typeList.begin()));
        shift = 1;
    }

    Lanelet newLanelet = Lanelet(newId, leftVertices, rightVertices, {}, {}, typeList, userOneWay, userBidirectional);
    std::shared_ptr<Lane> lane = std::make_shared<Lane>(containedLanelets, newLanelet);

    return lane;
}

std::vector<std::shared_ptr<Lanelet>>
lane_operations::extractLaneletsFromLanes(const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::unordered_set<size_t> ids;
    auto lanelets = lanes |
                    ranges::views::transform([](const auto &lane) { return lane->getContainedLanelets(); })
                    // Join vector of vectors into vector
                    | ranges::actions::join;

    return lanelets
           // Only keep lanelets we haven't seen yet
           | ranges::views::filter([&ids](const auto &lanelet) {
                 const auto lid = lanelet->getId();
                 bool not_exists = ids.count(lid) == 0;
                 if (not_exists)
                     ids.insert(lid);
                 return not_exists;
             }) |
           ranges::to<std::vector>;
}

std::vector<std::shared_ptr<Lanelet>>
lane_operations::combineLaneLanelets(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &lanes) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lane : lanes)
        for (const auto &let : lane)
            if (!(std::any_of(lanelets.begin(), lanelets.end(),
                              [let](const std::shared_ptr<Lanelet> &exLet) { return exLet->getId() == let->getId(); })))
                lanelets.push_back(let);
    return lanelets;
}
