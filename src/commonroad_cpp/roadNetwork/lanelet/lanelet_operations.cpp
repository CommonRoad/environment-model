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

bool lanelet_operations::checkClassifyingLaneletTypeTemplate(std::tuple<LaneletType,LaneletType, LaneletType> laneletTypeTemplate, std::set<LaneletType> typesLaneletOne, std::set<LaneletType> typesLaneletTwo, std::set<LaneletType> typesLaneletThree){
    return (containsLaneletType(std::get<0>(laneletTypeTemplate), typesLaneletOne) or std::get<0>(laneletTypeTemplate) == LaneletType::unknown) and
(containsLaneletType(std::get<1>(laneletTypeTemplate), typesLaneletTwo) or std::get<1>(laneletTypeTemplate) == LaneletType::unknown) and
(containsLaneletType(std::get<2>(laneletTypeTemplate), typesLaneletThree) or std::get<2>(laneletTypeTemplate) == LaneletType::unknown);
}

bool lanelet_operations::checkClassifyingLaneletTypeTemplate(std::tuple<LaneletType,LaneletType, LaneletType> laneletTypeTemplate, std::set<LaneletType> typesLaneletOne, std::set<LaneletType> typesLaneletTwo){
    return (containsLaneletType(std::get<0>(laneletTypeTemplate), typesLaneletOne) or std::get<0>(laneletTypeTemplate) == LaneletType::unknown) and
        (containsLaneletType(std::get<1>(laneletTypeTemplate), typesLaneletTwo) or std::get<1>(laneletTypeTemplate) == LaneletType::unknown);
}

bool lanelet_operations::checkClassifyingLaneletTypeTemplate(std::tuple<LaneletType,LaneletType, LaneletType> laneletTypeTemplate, std::set<LaneletType> typesLaneletOne){
    return (containsLaneletType(std::get<0>(laneletTypeTemplate), typesLaneletOne) or std::get<0>(laneletTypeTemplate) == LaneletType::unknown);
}

bool lanelet_operations::containsClassifyingLaneletType(std::shared_ptr<Lanelet> lanelet, std::set<std::tuple<LaneletType,LaneletType, LaneletType>> classifyingLaneletTypes){
    if(lanelet->getPredecessors().empty() and !lanelet->getSuccessors().empty()) {
        for (const auto &types : classifyingLaneletTypes)
            for (const auto &suc : lanelet->getPredecessors())
                if (checkClassifyingLaneletTypeTemplate(types, lanelet->getLaneletTypes(), suc->getLaneletTypes()))
                    return true;
    } else if(lanelet->getSuccessors().empty() and !lanelet->getPredecessors().empty()) {
        for (const auto &types : classifyingLaneletTypes)
            for (const auto &pre : lanelet->getSuccessors())
                if (checkClassifyingLaneletTypeTemplate(types, pre->getLaneletTypes(), lanelet->getLaneletTypes()))
                    return true;
    }
    else if(!lanelet->getSuccessors().empty() and !lanelet->getPredecessors().empty()) {
        for (const auto &pre : lanelet->getPredecessors())
            for (const auto &types : classifyingLaneletTypes)
                for (const auto &suc : lanelet->getSuccessors())
                    if (checkClassifyingLaneletTypeTemplate(types, pre->getLaneletTypes(), lanelet->getLaneletTypes(),
                                                            suc->getLaneletTypes()))
                        return true;
    } else {
        for (const auto &types : classifyingLaneletTypes)
            if (checkClassifyingLaneletTypeTemplate(types, lanelet->getLaneletTypes()))
                    return true;
    }
    return false;
}


std::vector<std::vector<std::shared_ptr<Lanelet>>> lanelet_operations::combineLaneletAndSuccessorsWithSameTypeToLane(
    const std::shared_ptr<Lanelet> &curLanelet, std::set<std::tuple<LaneletType, LaneletType, LaneletType>> classifyingLaneletTypes,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets, bool startLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);

    // check whether it is the last lanelet of the lane, the lane contains no loop, and classifying lanelet type is not reached
    if (!curLanelet->getSuccessors().empty() and !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); })
        and (!containsClassifyingLaneletType(curLanelet, classifyingLaneletTypes) or laneletList.size() == 1)) {
        for (const auto &la : curLanelet->getSuccessors()) {
            if(containsClassifyingLaneletType(la, classifyingLaneletTypes) and startLanelets)
                continue;
            auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, classifyingLaneletTypes, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>> lanelet_operations::combineLaneletAndPredecessorsWithSameTypeToLane(
    const std::shared_ptr<Lanelet> &curLanelet, std::set<std::tuple<LaneletType, LaneletType, LaneletType>> classifyingLaneletTypes,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets, bool startLanelets) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);

    // check whether it is the last lanelet of the lane, the lane contains no loop, and classifying lanelet type is not reached
    if (!curLanelet->getPredecessors().empty() and !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); })
        and (!containsClassifyingLaneletType(curLanelet, classifyingLaneletTypes) or laneletList.size() == 1)) {
        for (const auto &la : curLanelet->getPredecessors()) {
            if(containsClassifyingLaneletType(curLanelet, classifyingLaneletTypes) and startLanelets)
                continue;
            auto newLanes{combineLaneletAndPredecessorsWithSameTypeToLane(la, classifyingLaneletTypes, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lanelet_operations::createInterstateLanes(const std::vector<std::shared_ptr<Lanelet>> &network, size_t newId) {
    std::vector<std::shared_ptr<Lane>> lanes;
    std::vector<std::shared_ptr<Lanelet>> startLanelets;
    std::set<std::tuple<LaneletType, LaneletType, LaneletType>> classifyingLaneletTypes{{LaneletType::mainCarriageWay, LaneletType::shoulder, LaneletType::unknown},
                                                  {LaneletType::mainCarriageWay, LaneletType::accessRamp, LaneletType::unknown},
                                                  {LaneletType::mainCarriageWay, LaneletType::exitRamp, LaneletType::unknown}};

    for (const auto &la : network)
        if (la->getPredecessors().empty()) // if no predecessor -> use as start lanelet
            startLanelets.push_back(la);
    // check for classifying lanelet types
        else if(containsClassifyingLaneletType(la, classifyingLaneletTypes))
            startLanelets.push_back(la);

    // create lanes
    for (const auto &la : startLanelets) {
        auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, classifyingLaneletTypes, {}, true)};
        for (const auto &newLane : newLanes)
            lanes.push_back(createLaneByContainedLanelets(newLane, newId++));
    }
    return lanes;
}

std::vector<std::shared_ptr<Lane>> lanelet_operations::createLanesBySingleLanelets(
    const std::vector<std::shared_ptr<Lanelet>> &initialLanelets, size_t newId,
    const std::map<std::set<size_t>, std::tuple<std::set<size_t>, std::shared_ptr<Lane>>> &existingLanes) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &la : initialLanelets) {
        // lane was already created based on this initial lanelet -> continue with next lanelet
        bool laneExists{false};
        for (const auto &lane : existingLanes) {
            if (lane.first.find(la->getId()) != lane.first.end() and
                std::get<0>(existingLanes.at(lane.first)).find(la->getId()) !=
                    std::get<0>(existingLanes.at(lane.first)).end()) {
                lanes.push_back(std::get<1>(existingLanes.at(lane.first)));
                laneExists = true;
            }
        }
        if (laneExists)
            continue;

        std::set<std::tuple<LaneletType, LaneletType, LaneletType>> classifyingLaneletTypesSuccessor;
        std::set<std::tuple<LaneletType, LaneletType, LaneletType>> classifyingLaneletTypesPredecessor;
        if (containsLaneletType(LaneletType::incoming, la->getLaneletTypes())) {
            classifyingLaneletTypesPredecessor.insert({LaneletType::unknown, LaneletType::intersection, LaneletType::unknown});
            classifyingLaneletTypesSuccessor.insert({LaneletType::unknown, LaneletType::incoming, LaneletType::unknown});
        } else if (containsLaneletType(LaneletType::intersection, la->getLaneletTypes())) {
            classifyingLaneletTypesPredecessor.insert({LaneletType::unknown, LaneletType::incoming, LaneletType::unknown});
            classifyingLaneletTypesSuccessor.insert({LaneletType::incoming, LaneletType::unknown, LaneletType::unknown});
        } else {
            classifyingLaneletTypesPredecessor.insert({LaneletType::unknown, LaneletType::incoming, LaneletType::unknown});
            classifyingLaneletTypesSuccessor.insert({LaneletType::intersection, LaneletType::unknown, LaneletType::unknown});
        }
        auto newLaneSuccessorParts{combineLaneletAndSuccessorsWithSameTypeToLane(la, classifyingLaneletTypesSuccessor)};
        auto newLanePredecessorParts{combineLaneletAndPredecessorsWithSameTypeToLane(la, classifyingLaneletTypesPredecessor)};
        if (!newLaneSuccessorParts.empty() and !newLanePredecessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                for (const auto &lanePre : newLanePredecessorParts) {
                    std::vector<std::shared_ptr<Lanelet>> containedLanelets{lanePre};
                    std::reverse(containedLanelets.begin(), containedLanelets.end());
                    containedLanelets.insert(containedLanelets.end(), laneSuc.begin() + 1, laneSuc.end());
                    auto newLane{createLaneByContainedLanelets(containedLanelets, newId++)};
                    if (!std::any_of(lanes.begin(), lanes.end(), [newLane](std::shared_ptr<Lane> existingLane) {
                            return newLane->getContainedLaneletIDs() == existingLane->getContainedLaneletIDs();
                        }))
                        lanes.push_back(newLane);
                }
        else if (!newLaneSuccessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts) {
                auto newLane{createLaneByContainedLanelets(laneSuc, newId++)};
                if (!std::any_of(lanes.begin(), lanes.end(), [newLane](std::shared_ptr<Lane> existingLane) {
                        return newLane->getContainedLaneletIDs() == existingLane->getContainedLaneletIDs();
                    }))
                    lanes.push_back(newLane);
            }
        else
            for (const auto &lanePre : newLanePredecessorParts) {
                auto newLane{createLaneByContainedLanelets(lanePre, newId++)};
                if (!std::any_of(lanes.begin(), lanes.end(), [newLane](std::shared_ptr<Lane> existingLane) {
                        return newLane->getContainedLaneletIDs() == existingLane->getContainedLaneletIDs();
                    }))
                    lanes.push_back(newLane);
            }
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
    return baseTypesSet.find(type) != baseTypesSet.end();
}

std::set<size_t> lanelet_operations::extractIds(std::vector<std::shared_ptr<Lanelet>> lanelets) {
    std::set<size_t> ids;
    for (const auto &la : lanelets)
        ids.insert(la->getId());
    return ids;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{lanelet};

    while (curLanelet->getAdjacentRight().adj != nullptr) {
        adjacentLanelets.push_back(curLanelet->getAdjacentRight().adj);
        curLanelet = curLanelet->getAdjacentRight().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{lanelet};

    while (curLanelet->getAdjacentLeft().adj != nullptr) {
        adjacentLanelets.push_back(curLanelet->getAdjacentLeft().adj);
        curLanelet = curLanelet->getAdjacentLeft().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::adjacentLanelets(std::shared_ptr<Lanelet> lanelet) {
    std::vector<std::shared_ptr<Lanelet>> relevantLanelets{lanelet};
    auto leftLanelets{laneletsLeftOfLanelet(lanelet)};
    auto rightLanelets{laneletsRightOfLanelet(lanelet)};
    relevantLanelets.insert(relevantLanelets.end(), leftLanelets.begin(), leftLanelets.end());
    relevantLanelets.insert(relevantLanelets.end(), rightLanelets.begin(), rightLanelets.end());
    return relevantLanelets;
}

bool lanelet_operations::adjacentLanes(std::shared_ptr<Lane> laneOne, std::shared_ptr<Lane> laneTwo,
                                       std::vector<std::shared_ptr<Lanelet>> relevantLanelets) {
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