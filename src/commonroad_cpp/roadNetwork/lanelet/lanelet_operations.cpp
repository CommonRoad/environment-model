//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include <algorithm>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <utility>

#include <range/v3/all.hpp>

LaneletType lanelet_operations::matchStringToLaneletType(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    str.erase(std::remove_if(str.begin(), str.end(), [](char elem) { return elem == '_'; }), str.end());
    if (LaneletTypeNames.count(str) == 1)
        return LaneletTypeNames.at(str);
    else
        throw std::logic_error("lanelet_operations::matchStringToLaneletType: Invalid lanelet type!");
}

DrivingDirection lanelet_operations::matchStringToDrivingDirection(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    if (DrivingDirectionNames.count(str) == 1)
        return DrivingDirectionNames.at(str);
    else
        return DrivingDirection::invalid;
}

LineMarking lanelet_operations::matchStringToLineMarking(const std::string &type) {
    if (type == "solid")
        return LineMarking::solid;
    else if (type == "dashed")
        return LineMarking::dashed;
    else if (type == "broad_solid")
        return LineMarking::broad_solid;
    else if (type == "broad_dashed")
        return LineMarking::broad_dashed;
    else if (type == "no_marking")
        return LineMarking::no_marking;
    else
        return LineMarking::unknown;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>>
lanelet_operations::combineLaneletAndSuccessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov,
                                                      int numIntersections,
                                                      const std::vector<std::shared_ptr<Lanelet>> &containedLanelets) {
    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{-laneletList.at(0)->getPathLength().back()}; // neglect initial lanelet
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
            auto newLanes{combineLaneletAndSuccessorsToLane(lanelet, fov, numIntersections, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>>
lanelet_operations::combineLaneletAndPredecessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov,
                                                        int numIntersections,
                                                        std::vector<std::shared_ptr<Lanelet>> containedLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{-laneletList.at(0)->getPathLength().back()}; // neglect initial lanelet
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
            auto newLanes{combineLaneletAndPredecessorsToLane(lanelet, fov, numIntersections, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lanelet_operations::createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                                                const std::shared_ptr<RoadNetwork> &roadNetwork, double fovRear,
                                                double fovFront, int numIntersections) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &lanelet : initialLanelets) {
        // lane was already created based on this initial lanelet -> continue with next lanelet
        auto newLanes{roadNetwork->findLanesByBaseLanelet(lanelet->getId())};
        if (!newLanes.empty()) {
            for (const auto &newLane : newLanes)
                if (!std::any_of(lanes.begin(), lanes.end(), [newLane](const std::shared_ptr<Lane> &lane) {
                        return newLane->getContainedLaneletIDs() == lane->getContainedLaneletIDs();
                    }))
                    lanes.push_back(newLane);
            continue;
        }

        auto newLaneSuccessorParts{combineLaneletAndSuccessorsToLane(lanelet, fovFront, numIntersections)};
        auto newLanePredecessorParts{combineLaneletAndPredecessorsToLane(lanelet, fovRear, numIntersections)};
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
lanelet_operations::createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
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

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                                                 bool sameDirection) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentRight().adj != nullptr and
           !std::any_of(adjacentLanelets.begin(), adjacentLanelets.end(),
                        [curLanelet](const std::shared_ptr<Lanelet> &let) {
                            return let->getId() == curLanelet->getAdjacentRight().adj->getId();
                        }) and
           (!sameDirection or curLanelet->getAdjacentRight().dir == DrivingDirection::same)) {
        adjacentLanelets.push_back(curLanelet->getAdjacentRight().adj);
        curLanelet = curLanelet->getAdjacentRight().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                                                bool sameDirection) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentLeft().adj != nullptr and
           !std::any_of(adjacentLanelets.begin(), adjacentLanelets.end(),
                        [curLanelet](const std::shared_ptr<Lanelet> &let) {
                            return let->getId() == curLanelet->getAdjacentLeft().adj->getId();
                        }) and
           (!sameDirection or curLanelet->getAdjacentLeft().dir == DrivingDirection::same)) {
        adjacentLanelets.push_back(curLanelet->getAdjacentLeft().adj);
        curLanelet = curLanelet->getAdjacentLeft().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet,
                                                                           bool sameDirection) {
    auto leftLanelets{laneletsLeftOfLanelet(lanelet, sameDirection)};
    auto rightLanelets{laneletsRightOfLanelet(lanelet, sameDirection)};
    std::initializer_list<std::shared_ptr<Lanelet>> initial_lanelet{lanelet};

    // remove duplicates
    std::vector<std::shared_ptr<Lanelet>> adjLanelets =
        ranges::concat_view(initial_lanelet, leftLanelets, rightLanelets) | ranges::to<std::vector>;
    sort(adjLanelets.begin(), adjLanelets.end());
    adjLanelets.erase(unique(adjLanelets.begin(), adjLanelets.end()), adjLanelets.end());
    return adjLanelets;
}

bool lanelet_operations::areLaneletsInDirectlyAdjacentLanes(
    const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
    const std::vector<std::shared_ptr<Lanelet>> &relevantLanelets) {
    for (const auto &la1 : relevantLanelets) {
        for (const auto &la2 : relevantLanelets) {
            if (la1->getId() == la2->getId())
                continue;
            if ((la1->getAdjacentRight().adj != nullptr and la1->getAdjacentRight().adj->getId() == la2->getId()) or
                (la1->getAdjacentLeft().adj != nullptr and la1->getAdjacentLeft().adj->getId() == la2->getId()))
                if ((laneOne->getContainedLaneletIDs().find(la1->getId()) != laneOne->getContainedLaneletIDs().end() and
                     laneTwo->getContainedLaneletIDs().find(la2->getId()) != laneTwo->getContainedLaneletIDs().end()) or
                    (laneTwo->getContainedLaneletIDs().find(la1->getId()) != laneTwo->getContainedLaneletIDs().end() and
                     laneOne->getContainedLaneletIDs().find(la2->getId()) != laneOne->getContainedLaneletIDs().end()))
                    return true;
        }
    }
    return false;
}

double lanelet_operations::roadWidth(const std::shared_ptr<Lanelet> &lanelet, double xPosition, double yPosition) {
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets = adjacentLanelets(lanelet, false);

    auto lanelet_widths = adj_lanelets | ranges::views::transform([xPosition, yPosition](auto &adjLanelet) -> double {
                              return adjLanelet->getWidth(xPosition, yPosition);
                          });

    return ranges::accumulate(lanelet_widths, 0.0);
}

std::vector<std::shared_ptr<TrafficLight>>
lanelet_operations::activeTlsByLanelet(size_t timeStep, const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getTrafficLights() | ranges::views::filter([timeStep](const auto &light) {
               return light->isActive() or light->getElementAtTime(timeStep).color != TrafficLightState::inactive;
           }) |
           ranges::to<std::vector>;
}

std::vector<std::shared_ptr<Lanelet>>
lanelet_operations::extractLaneletsFromLanes(const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::unordered_set<size_t> ids;
    auto lanelets = lanes |
                    ranges::views::transform([](const auto &lane) { return lane->getContainedLanelets(); })
                    // Join vector of vectors into vector
                    | ranges::actions::join;

    return lanelets
           // Only keep lanelets we haven't seen yet
           | ranges::views::filter([&ids](const auto &lanelet) {
                 const auto id = lanelet->getId();
                 bool not_exists = ids.count(id) == 0;
                 if (not_exists)
                     ids.insert(id);
                 return not_exists;
             }) |
           ranges::to<std::vector>;
}

std::vector<std::shared_ptr<Lanelet>>
lanelet_operations::combineLaneLanelets(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &lanes) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lane : lanes)
        for (const auto &let : lane)
            if (!(std::any_of(lanelets.begin(), lanelets.end(),
                              [let](const std::shared_ptr<Lanelet> &exLet) { return exLet->getId() == let->getId(); })))
                lanelets.push_back(let);
    return lanelets;
}
