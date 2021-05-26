//
// Created by Sebastian Maierhofer on 09.11.20.
//

#include "lanelet_operations.h"

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
combineLaneletAndSuccessorsWithSameTypeToLane(std::shared_ptr<Lanelet> curLanelet, Lanelet curLaneLanelet,
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
    if (curLaneLanelet.getCenterVertices().size() == 0) {
        userOneWay = curLanelet->getUserOneWay();
        userBidirectional = curLanelet->getUserBidirectional();
        centerVertices = curLanelet->getCenterVertices();
        leftBorderVertices = curLanelet->getLeftBorderVertices();
        rightBorderVertices = curLanelet->getRightBorderVertices();
        predecessorLanelets = curLanelet->getPredecessors();
        typeList = curLanelet->getLaneletType();
    } else {
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

    if (curLanelet->getSuccessors().size() > 0) {
        // create new lane
        Lanelet newLanelet = Lanelet(id, leftBorderVertices, rightBorderVertices, predecessorLanelets,
                                     successorLanelets, typeList, userOneWay, userBidirectional);
        for (const auto &la : curLanelet->getSuccessors()) {
            auto newLanes{combineLaneletAndSuccessorsWithSameTypeToLane(la, newLanelet, laneletList)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
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
