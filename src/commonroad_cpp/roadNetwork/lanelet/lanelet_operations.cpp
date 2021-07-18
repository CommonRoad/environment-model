//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "lanelet_operations.h"
#include <algorithm>
#include <utility>

DrivingDirection lanelet_operations::matchStringToDrivingDirection(const std::string &type) {
    if (type == "same")
        return DrivingDirection::same;
    else if (type == "opposite")
        return DrivingDirection::opposite;
    else
        return DrivingDirection::invalid;
}

std::string lanelet_operations::matchDrivingDirectionToString(const DrivingDirection &type) {
    if (type == DrivingDirection::same)
        return "same";
    else if (type == DrivingDirection::opposite)
        return "opposite";
    else
        return "invalid";
}

LaneletType lanelet_operations::matchStringToLaneletType(const std::string &type) {
    if (type == "interstate")
        return LaneletType::interstate;
    else if (type == "urban")
        return LaneletType::urban;
    else if (type == "crosswalk")
        return LaneletType::crosswalk;
    else if (type == "busStop")
        return LaneletType::busStop;
    else if (type == "country")
        return LaneletType::country;
    else if (type == "highway")
        return LaneletType::highway;
    else if (type == "driveWay")
        return LaneletType::driveWay;
    else if (type == "mainCarriageWay")
        return LaneletType::mainCarriageWay;
    else if (type == "accessRamp")
        return LaneletType::accessRamp;
    else if (type == "exitRamp")
        return LaneletType::exitRamp;
    else if (type == "shoulder")
        return LaneletType::shoulder;
    else if (type == "bikeLane")
        return LaneletType::bikeLane;
    else if (type == "sidewalk")
        return LaneletType::sidewalk;
    else if (type == "busLane")
        return LaneletType::busLane;
    else if (type == "intersection")
        return LaneletType::intersection;
    else
        return LaneletType::unknown;
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
                                                      std::vector<std::shared_ptr<Lanelet>> containedLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{0};
    for (const auto &la : laneletList)
        laneLength += la->getPathLength().back();

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getSuccessors().empty() and
        !std::any_of(
            containedLanelets.begin(), containedLanelets.end(),
            [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); }) and
        laneLength < fov) {
        for (const auto &la : curLanelet->getSuccessors()) {
            auto newLanes{combineLaneletAndSuccessorsToLane(la, fov, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>>
lanelet_operations::combineLaneletAndPredecessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov,
                                                        std::vector<std::shared_ptr<Lanelet>> containedLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{0};
    for (const auto &la : laneletList)
        laneLength += la->getPathLength().back();

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getPredecessors().empty() and
        !std::any_of(
            containedLanelets.begin(), containedLanelets.end(),
            [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); }) and
        laneLength < fov) {
        for (const auto &la : curLanelet->getPredecessors()) {
            auto newLanes{combineLaneletAndPredecessorsToLane(la, fov, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lanelet_operations::createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                                                std::shared_ptr<size_t> idCounter,
                                                const std::shared_ptr<RoadNetwork> &roadNetwork, double fov) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &la : initialLanelets) {
        // lane was already created based on this initial lanelet -> continue with next lanelet
        auto newLanes{roadNetwork->findLanesSpannedByLanelet(la->getId())};
        if (!newLanes.empty()) {
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
            continue;
        }

        auto newLaneSuccessorParts{combineLaneletAndSuccessorsToLane(la, fov)};
        auto newLanePredecessorParts{combineLaneletAndPredecessorsToLane(la, fov)};
        if (!newLaneSuccessorParts.empty() and !newLanePredecessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                for (const auto &lanePre : newLanePredecessorParts) {
                    std::vector<std::shared_ptr<Lanelet>> containedLanelets{lanePre};
                    std::reverse(containedLanelets.begin(), containedLanelets.end());
                    containedLanelets.insert(containedLanelets.end(), laneSuc.begin() + 1, laneSuc.end());
                    newLanes.push_back(createLaneByContainedLanelets(containedLanelets, ++*idCounter));
                }
        else if (!newLaneSuccessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts) {
                newLanes.push_back(createLaneByContainedLanelets(laneSuc, ++*idCounter));
            }
        else
            for (const auto &lanePre : newLanePredecessorParts) {
                newLanes.push_back(createLaneByContainedLanelets(lanePre, ++*idCounter));
            }
        newLanes = roadNetwork->addLanes(newLanes, la->getId());
        for (const auto &newLane : newLanes)
            if (!std::any_of(lanes.begin(), lanes.end(), [newLane](const std::shared_ptr<Lane> &lane) {
                    return newLane->getId() == lane->getId();
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

    for (const auto &la : containedLanelets) {
        std::set_intersection(userOneWay.begin(), userOneWay.end(), la->getUsersOneWay().begin(),
                              la->getUsersOneWay().end(), std::inserter(userOneWay, userOneWay.begin()));

        std::set_intersection(userBidirectional.begin(), userBidirectional.end(), la->getUsersBidirectional().begin(),
                              la->getUsersBidirectional().end(),
                              std::inserter(userBidirectional, userBidirectional.begin()));

        centerVertices.insert(centerVertices.end(), la->getCenterVertices().begin() + shift,
                              la->getCenterVertices().end());

        leftVertices.insert(leftVertices.end(), la->getLeftBorderVertices().begin() + shift,
                            la->getLeftBorderVertices().end());

        rightVertices.insert(rightVertices.end(), la->getRightBorderVertices().begin() + shift,
                             la->getRightBorderVertices().end());

        std::set_intersection(typeList.begin(), typeList.end(), la->getLaneletTypes().begin(),
                              la->getLaneletTypes().end(), std::inserter(typeList, typeList.begin()));
        shift = 1;
    }

    Lanelet newLanelet = Lanelet(newId, leftVertices, rightVertices, {}, {}, typeList, userOneWay, userBidirectional);
    std::shared_ptr<Lane> lane = std::make_shared<Lane>(containedLanelets, newLanelet);

    return lane;
}

std::shared_ptr<Lane> lanelet_operations::mergeLanes(const std::shared_ptr<Lane> &predecessorLane,
                                                     const std::shared_ptr<Lane> &successorLane, size_t newId) {
    std::set<ObstacleType> userOneWay;
    std::set_intersection(predecessorLane->getUsersOneWay().begin(), predecessorLane->getUsersOneWay().end(),
                          successorLane->getUsersOneWay().begin(), successorLane->getUsersOneWay().end(),
                          std::inserter(userOneWay, userOneWay.begin()));
    std::set<ObstacleType> userBidirectional;
    std::set_intersection(predecessorLane->getUsersBidirectional().begin(),
                          predecessorLane->getUsersBidirectional().end(),
                          successorLane->getUsersBidirectional().begin(), successorLane->getUsersBidirectional().end(),
                          std::inserter(userBidirectional, userBidirectional.begin()));
    auto centerVertices = predecessorLane->getCenterVertices();
    centerVertices.insert(centerVertices.end(), successorLane->getCenterVertices().begin() + 1,
                          successorLane->getCenterVertices().end());
    auto leftBorderVertices = predecessorLane->getLeftBorderVertices();
    leftBorderVertices.insert(leftBorderVertices.end(), successorLane->getLeftBorderVertices().begin() + 1,
                              successorLane->getLeftBorderVertices().end());
    auto rightBorderVertices = predecessorLane->getRightBorderVertices();
    rightBorderVertices.insert(rightBorderVertices.end(), successorLane->getRightBorderVertices().begin() + 1,
                               successorLane->getRightBorderVertices().end());
    auto predecessorLanelets = predecessorLane->getPredecessors();
    auto successorLanelets = successorLane->getSuccessors();
    std::set<LaneletType> typeList;
    std::set_intersection(predecessorLane->getLaneletTypes().begin(), predecessorLane->getLaneletTypes().end(),
                          successorLane->getLaneletTypes().begin(), successorLane->getLaneletTypes().end(),
                          std::inserter(typeList, typeList.begin()));
    std::vector<std::shared_ptr<Lanelet>> containedLanelets;
    containedLanelets.insert(containedLanelets.begin(), predecessorLane->getContainedLanelets().begin(),
                             predecessorLane->getContainedLanelets().end());
    containedLanelets.insert(containedLanelets.begin(), successorLane->getContainedLanelets().begin(),
                             successorLane->getContainedLanelets().end());

    Lanelet newLanelet = Lanelet(newId, leftBorderVertices, rightBorderVertices, predecessorLanelets, successorLanelets,
                                 typeList, userOneWay, userBidirectional);
    std::shared_ptr<Lane> lane = std::make_shared<Lane>(containedLanelets, newLanelet);

    return lane;
}

bool lanelet_operations::containsLaneletType(LaneletType type, std::set<LaneletType> baseTypesSet) {
    return baseTypesSet.find(type) != baseTypesSet.end();
}

std::vector<size_t> lanelet_operations::extractIds(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<size_t> ids;
    for (const auto &la : lanelets)
        ids.push_back(la->getId());
    return ids;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentRight().adj != nullptr and
           curLanelet->getAdjacentLeft().dir != DrivingDirection::opposite) {
        adjacentLanelets.push_back(curLanelet->getAdjacentRight().adj);
        curLanelet = curLanelet->getAdjacentRight().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentLeft().adj != nullptr and
           curLanelet->getAdjacentLeft().dir != DrivingDirection::opposite) {
        adjacentLanelets.push_back(curLanelet->getAdjacentLeft().adj);
        curLanelet = curLanelet->getAdjacentLeft().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet) {
    std::vector<std::shared_ptr<Lanelet>> relevantLanelets{lanelet};
    auto leftLanelets{laneletsLeftOfLanelet(lanelet)};
    auto rightLanelets{laneletsRightOfLanelet(lanelet)};
    relevantLanelets.insert(relevantLanelets.end(), leftLanelets.begin(), leftLanelets.end());
    relevantLanelets.insert(relevantLanelets.end(), rightLanelets.begin(), rightLanelets.end());
    return relevantLanelets;
}

bool lanelet_operations::adjacentLanes(const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
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