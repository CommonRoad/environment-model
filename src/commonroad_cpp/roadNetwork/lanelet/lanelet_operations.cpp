//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "lanelet_operations.h"

DrivingDirection matchStringToDrivingDirection(const std::string &type) {
    if (type == "same")
        return DrivingDirection::same;
    else if (type == "opposite")
        return DrivingDirection::opposite;
    else
        return DrivingDirection::invalid;
}

std::string matchDrivingDirectionToString(const DrivingDirection &type) {
    if (type == DrivingDirection::same)
        return "same";
    else if (type == DrivingDirection::opposite)
        return "opposite";
    else
        return "invalid";
}

LaneletType matchStringToLaneletType(const std::string &type) {
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

LineMarking matchStringToLineMarking(const std::string &type) {
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

std::vector<std::shared_ptr<Lane>>
combineLaneletAndSuccessorsWithSameTypeToLane(const std::shared_ptr<Lanelet> &curLanelet, const Lanelet &curLaneLanelet,
                                              std::vector<std::shared_ptr<Lanelet>> containedLanelets) {
    // initialize lanelet elements
    static size_t id;
    std::vector<std::shared_ptr<Lane>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    std::set<ObstacleType> userOneWay;
    std::set<ObstacleType> userBidirectional;
    std::vector<vertex> centerVertices;
    std::vector<vertex> leftBorderVertices;
    std::vector<vertex> rightBorderVertices;
    std::vector<std::shared_ptr<Lanelet>> predecessorLanelets;
    std::vector<std::shared_ptr<Lanelet>> successorLanelets{curLanelet->getSuccessors()};
    std::set<LaneletType> typeList;
    laneletList.push_back(curLanelet);
    if (curLaneLanelet.getCenterVertices().empty()) { // first lanelet of lane
        userOneWay = curLanelet->getUserOneWay();
        userBidirectional = curLanelet->getUserBidirectional();
        centerVertices = curLanelet->getCenterVertices();
        leftBorderVertices = curLanelet->getLeftBorderVertices();
        rightBorderVertices = curLanelet->getRightBorderVertices();
        predecessorLanelets = curLanelet->getPredecessors();
        typeList = curLanelet->getLaneletType();
    } else { // merge with predecessor lanelets
        userOneWay = curLaneLanelet.getUserOneWay();
        userBidirectional = curLaneLanelet.getUserBidirectional();
        centerVertices = curLaneLanelet.getCenterVertices();
        centerVertices.insert(centerVertices.end(), curLanelet->getCenterVertices().begin() + 1,
                              curLanelet->getCenterVertices().end());
        leftBorderVertices = curLaneLanelet.getLeftBorderVertices();
        leftBorderVertices.insert(leftBorderVertices.end(), curLanelet->getLeftBorderVertices().begin() + 1,
                                  curLanelet->getLeftBorderVertices().end());
        rightBorderVertices = curLaneLanelet.getRightBorderVertices();
        rightBorderVertices.insert(rightBorderVertices.end(), curLanelet->getRightBorderVertices().begin() + 1,
                                   curLanelet->getRightBorderVertices().end());
        predecessorLanelets = curLaneLanelet.getPredecessors();
        typeList = curLaneLanelet.getLaneletType();
    }

    std::set<LaneletType> classifyingLaneletTypes{LaneletType::incoming, LaneletType::shoulder, LaneletType::accessRamp,
                                                  LaneletType::exitRamp}; // TODO find common place for storage
    // check whether it is the last lanelet of the lane
    if (!curLanelet->getSuccessors().empty() and
        !std::any_of(curLanelet->getLaneletType().begin(), curLanelet->getLaneletType().end(),
                     [](LaneletType t) { return t == LaneletType::incoming; }) and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &la) { return curLanelet->getId() == la->getId(); })) {

        // create new lane
        Lanelet newLanelet = Lanelet(id, leftBorderVertices, rightBorderVertices, predecessorLanelets,
                                     successorLanelets, typeList, userOneWay, userBidirectional);

        std::set<LaneletType> intersectingLa;
        std::set_intersection(curLanelet->getLaneletType().begin(), curLanelet->getLaneletType().end(),
                              classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                              std::inserter(intersectingLa, intersectingLa.begin()));
        for (const auto &la : curLanelet->getSuccessors()) {
            std::set<LaneletType> intersectingSuc;
            std::set_intersection(la->getLaneletType().begin(), la->getLaneletType().end(),
                                  classifyingLaneletTypes.begin(), classifyingLaneletTypes.end(),
                                  std::inserter(intersectingSuc, intersectingSuc.begin()));

            if (intersectingLa == intersectingSuc) {
                auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, newLanelet, laneletList)};
                lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
            }
        }
    } else {
        Lanelet newLanelet = Lanelet(id++, leftBorderVertices, rightBorderVertices, predecessorLanelets,
                                     successorLanelets, typeList, userOneWay, userBidirectional);
        geometry::EigenPolyline reference_path;
        for (auto vert : centerVertices)
            reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));

        geometry::util::resample_polyline(reference_path, 2, reference_path);

        std::shared_ptr<Lane> lane =
            std::make_shared<Lane>(laneletList, newLanelet, CurvilinearCoordinateSystem(reference_path));
        lanes.push_back(lane);
    }
    return lanes;
}
