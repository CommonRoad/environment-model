//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lanelet_operations.h"

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

std::vector<std::vector<std::shared_ptr<Lanelet>>> lanelet_operations::combineLaneletAndSuccessorsWithSameTypeToLane(
    const std::shared_ptr<Lanelet> &curLanelet, std::set<LaneletType> classifyingLaneletTypes,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);

    // check whether it is the last lanelet of the lane
    if (!curLanelet->getSuccessors().empty() and classifyingLaneletTypes != curLanelet->getLaneletTypes() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); })) {
        std::set<LaneletType> intersectingLa;
        std::set_intersection(curLanelet->getLaneletTypes().begin(), curLanelet->getLaneletTypes().end(),
                              classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                              std::inserter(intersectingLa, intersectingLa.begin()));
        for (const auto &la : curLanelet->getSuccessors()) {
            std::set<LaneletType> intersectingSuc;
            std::set_intersection(la->getLaneletTypes().begin(), la->getLaneletTypes().end(),
                                  classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                                  std::inserter(intersectingSuc, intersectingSuc.begin()));

            if (intersectingLa == intersectingSuc) {
                auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, classifyingLaneletTypes, laneletList)};
                lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
            }
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>> lanelet_operations::combineLaneletAndPredecessorsWithSameTypeToLane(
    const std::shared_ptr<Lanelet> &curLanelet, std::set<LaneletType> classifyingLaneletTypes,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);

    // check whether it is the last lanelet of the lane
    if (!curLanelet->getPredecessors().empty() and classifyingLaneletTypes != curLanelet->getLaneletTypes() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); })) {

        std::set<LaneletType> intersectingLa;
        std::set_intersection(curLanelet->getLaneletTypes().begin(), curLanelet->getLaneletTypes().end(),
                              classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                              std::inserter(intersectingLa, intersectingLa.begin()));
        for (const auto &la : curLanelet->getPredecessors()) {
            std::set<LaneletType> intersectingSuc;
            std::set_intersection(la->getLaneletTypes().begin(), la->getLaneletTypes().end(),
                                  classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                                  std::inserter(intersectingSuc, intersectingSuc.begin()));

            if (intersectingLa == intersectingSuc) {
                auto newLanes{
                    combineLaneletAndPredecessorsWithSameTypeToLane(la, classifyingLaneletTypes, laneletList)};
                lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
            }
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lanelet_operations::createInterstateLanes(const std::vector<std::shared_ptr<Lanelet>> &network, size_t newId) {
    std::vector<std::shared_ptr<Lane>> lanes;
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    std::set<LaneletType> classifyinglaneletTypes{LaneletType::shoulder, LaneletType::accessRamp,
                                                  LaneletType::exitRamp};

    for (const auto &la : network) {
        if (la->getPredecessors().empty()) // if no predecessor -> use as start lanelet
            startLanelets.push_back(la);
        else {
            std::vector<std::shared_ptr<Lanelet>> predecessors;
            for (const std::shared_ptr<Lanelet> &pre : la->getPredecessors()) {
                predecessors.push_back(pre);
            }

            // if no predecessor with same classifying type exist -> use this lanelet as start lanelet
            std::set<LaneletType> intersectingLaTypes;
            std::set_intersection(la->getLaneletTypes().begin(), la->getLaneletTypes().end(),
                                  classifyinglaneletTypes.begin(), classifyinglaneletTypes.end(),
                                  std::inserter(intersectingLaTypes, intersectingLaTypes.begin()));
            for (const auto &pred : la->getPredecessors()) {
                std::set<LaneletType> intersectingTypesPred;
                std::set_intersection(pred->getLaneletTypes().begin(), pred->getLaneletTypes().end(),
                                      classifyinglaneletTypes.begin(), classifyinglaneletTypes.end(),
                                      std::inserter(intersectingTypesPred, intersectingTypesPred.begin()));
                if (intersectingTypesPred.empty() != intersectingLaTypes.empty()) {
                    startLanelets.push_back(la);
                    break;
                }
            }
        }
    }
    // create lanes
    for (const auto &la : startLanelets) {
        auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, classifyinglaneletTypes)};
        for (const auto &newLane : newLanes)
            lanes.push_back(createLaneByContainedLanelets(newLane, newId++));
    }
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lanelet_operations::createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                                                size_t newId) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &la : initialLanelets) {
        std::set<LaneletType> classifyinglaneletTypesSuccessor;
        std::set<LaneletType> classifyinglaneletTypesPredecessor;
        if (containsLaneletType(LaneletType::incoming, la->getLaneletTypes())) {
            classifyinglaneletTypesPredecessor = {LaneletType::intersection};
            classifyinglaneletTypesSuccessor = {LaneletType::intersection};
        } else if (containsLaneletType(LaneletType::intersection, la->getLaneletTypes())) {
            classifyinglaneletTypesPredecessor = {LaneletType::incoming};
            classifyinglaneletTypesSuccessor = {
                LaneletType::intersection}; // should be successor of intersection (outgoing)
        } else {
            classifyinglaneletTypesPredecessor = {LaneletType::incoming};
            classifyinglaneletTypesSuccessor = {
                LaneletType::intersection}; // should be successor of intersection (outgoing)
        }
        auto newLaneSuccessorParts{combineLaneletAndSuccessorsWithSameTypeToLane(la)};
        auto newLanePredecessorParts{combineLaneletAndPredecessorsWithSameTypeToLane(la)};
        if (!newLaneSuccessorParts.empty() and !newLanePredecessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                for (const auto &lanePre : newLanePredecessorParts) {
                    std::vector<std::shared_ptr<Lanelet>> containedLanelets{lanePre};
                    std::reverse(containedLanelets.begin(), containedLanelets.end());
                    containedLanelets.insert(containedLanelets.end(), laneSuc.begin() + 1, laneSuc.end());
                    lanes.push_back(createLaneByContainedLanelets(containedLanelets, newId++));
                }
        else if (!newLaneSuccessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                lanes.push_back(createLaneByContainedLanelets(laneSuc, newId++));
        else
            for (const auto &lanePre : newLanePredecessorParts)
                lanes.push_back(createLaneByContainedLanelets(lanePre, newId++));
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
    for (const auto &la : containedLanelets) {
        std::set_intersection(userOneWay.begin(), userOneWay.end(), la->getUsersOneWay().begin(),
                              la->getUsersOneWay().end(), std::inserter(userOneWay, userOneWay.begin()));

        std::set_intersection(userBidirectional.begin(), userBidirectional.end(), la->getUsersBidirectional().begin(),
                              la->getUsersBidirectional().end(),
                              std::inserter(userBidirectional, userBidirectional.begin()));

        centerVertices.insert(centerVertices.end(), la->getCenterVertices().begin() + 1, la->getCenterVertices().end());

        leftVertices.insert(leftVertices.end(), la->getLeftBorderVertices().begin() + 1,
                            la->getLeftBorderVertices().end());

        rightVertices.insert(rightVertices.end(), la->getRightBorderVertices().begin() + 1,
                             la->getRightBorderVertices().end());

        std::set_intersection(typeList.begin(), typeList.end(), la->getLaneletTypes().begin(),
                              la->getLaneletTypes().end(), std::inserter(typeList, typeList.begin()));
    }

    Lanelet newLanelet = Lanelet(newId, leftVertices, rightVertices, {}, {}, typeList, userOneWay, userBidirectional);
    geometry::EigenPolyline reference_path;
    for (auto vert : centerVertices)
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));

    geometry::util::resample_polyline(reference_path, 2, reference_path);

    std::shared_ptr<Lane> lane =
        std::make_shared<Lane>(containedLanelets, newLanelet, CurvilinearCoordinateSystem(reference_path));
    return lane;
}

std::shared_ptr<Lane> lanelet_operations::mergeLanes(std::shared_ptr<Lane> predecessorLane,
                                                     std::shared_ptr<Lane> successorLane, size_t newId) {
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
    geometry::EigenPolyline reference_path;
    for (auto vert : centerVertices)
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));

    geometry::util::resample_polyline(reference_path, 2, reference_path);

    std::shared_ptr<Lane> lane =
        std::make_shared<Lane>(containedLanelets, newLanelet, CurvilinearCoordinateSystem(reference_path));
    return lane;
}

bool lanelet_operations::containsLaneletType(LaneletType type, std::set<LaneletType> baseTypesSet) {
    return std::any_of(baseTypesSet.begin(), baseTypesSet.end(), [type](LaneletType t) { return t == type; });
}

std::set<size_t> lanelet_operations::extractIds(std::vector<std::shared_ptr<Lanelet>> lanelets) {
    std::set<size_t> ids;
    for (const auto &la : lanelets)
        ids.insert(la->getId());
    return ids;
}