//
// Created by Sebastian Maierhofer on 09.11.20.
//

#include "lanelet_operations.h"

LaneletType matchStringToLaneletType(std::string type) {
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

LineMarking matchStringToLineMarking(std::string type) {
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

std::shared_ptr<Lane> combineLaneletAndSuccessorsWithSameTypeToLane(std::shared_ptr<Lanelet> curLanelet,
                                                                    LaneletType type) {
    // assumption: successors of a single lanelet have different types

    // initialize lanelet elements
    int id = curLanelet->getId();
    std::vector<std::shared_ptr<Lanelet>> laneletList{curLanelet};
    std::vector<ObstacleType> userOneWay{curLanelet->getUserOneWay()};
    std::vector<ObstacleType> userBidirectional{curLanelet->getUserBidirectional()};
    std::vector<vertex> centerVertices = curLanelet->getCenterVertices();
    std::vector<vertex> leftBorderVertices = curLanelet->getLeftBorderVertices();
    std::vector<vertex> rightBorderVertices = curLanelet->getRightBorderVertices();
    std::vector<std::shared_ptr<Lanelet>> predecessorLanelets = curLanelet->getPredecessors();
    std::vector<std::shared_ptr<Lanelet>> successorLanelets{nullptr};
    std::vector<LaneletType> typeList{type};

    if (!curLanelet->getSuccessors().empty())
        while (curLanelet != nullptr) {
            // iterate through all successors and concatenate them
            for (const auto &la : curLanelet->getSuccessors()) {
                if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                                [type](LaneletType t) { return t == type; })) {
                    curLanelet = la;
                    laneletList.push_back(curLanelet);
                    id += curLanelet->getId();
                    centerVertices.insert(centerVertices.end(),
                                          curLanelet->getCenterVertices().begin() + 1,
                                          curLanelet->getCenterVertices().end());
                    leftBorderVertices.insert(leftBorderVertices.end(),
                                              curLanelet->getLeftBorderVertices().begin() + 1,
                                              curLanelet->getLeftBorderVertices().end());
                    rightBorderVertices.insert(rightBorderVertices.end(),
                                               curLanelet->getRightBorderVertices().begin() + 1,
                                               curLanelet->getRightBorderVertices().end());
                    successorLanelets = curLanelet->getSuccessors();
                    break;
                } else {
                    curLanelet = nullptr;
                }
            }
            if (curLanelet and curLanelet->getSuccessors().empty())
                curLanelet = nullptr;
        }

    // create new lane
    Lanelet newLanelet = Lanelet(id, leftBorderVertices, rightBorderVertices, predecessorLanelets,
                                 successorLanelets, typeList, userOneWay, userBidirectional);
    newLanelet.createCenterVertices();
    newLanelet.constructOuterPolygon();

    geometry::EigenPolyline reference_path;
    for (auto vert : centerVertices) {
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }
    std::shared_ptr<Lane> lane = std::make_shared<Lane>(laneletList,
                                                        newLanelet,
                                                        CurvilinearCoordinateSystem(reference_path));
    return lane;
}

