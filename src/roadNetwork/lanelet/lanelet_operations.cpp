//
// Created by Sebastian Maierhofer on 09.11.20.
//

#include "lanelet_operations.h"

LaneletType matchStringToLaneletType(const char *type) {
    if (!(strcmp(type, "interstate")))
        return LaneletType::interstate;
    else if (!(strcmp(type, "urban")))
        return LaneletType::urban;
    else if (!(strcmp(type, "crosswalk")))
        return LaneletType::crosswalk;
    else if (!(strcmp(type, "busStop")))
        return LaneletType::busStop;
    else if (!(strcmp(type, "country")))
        return LaneletType::country;
    else if (!(strcmp(type, "highway")))
        return LaneletType::highway;
    else if (!(strcmp(type, "driveWay")))
        return LaneletType::driveWay;
    else if (!(strcmp(type, "mainCarriageWay")))
        return LaneletType::mainCarriageWay;
    else if (!(strcmp(type, "accessRamp")))
        return LaneletType::accessRamp;
    else if (!(strcmp(type, "exitRamp")))
        return LaneletType::exitRamp;
    else if (!(strcmp(type, "shoulder")))
        return LaneletType::shoulder;
    else if (!(strcmp(type, "bikeLane")))
        return LaneletType::bikeLane;
    else if (!(strcmp(type, "sidewalk")))
        return LaneletType::sidewalk;
    else if (!(strcmp(type, "busLane")))
        return LaneletType::busLane;
    else
        return LaneletType::unknown;
}

LineMarking matchStringToLineMarking(const char *type) {
    if (!(strcmp(type, "solid")))
        return LineMarking::solid;
    else if (!(strcmp(type, "dashed")))
        return LineMarking::dashed;
    else if (!(strcmp(type, "broad_solid")))
        return LineMarking::broad_solid;
    else if (!(strcmp(type, "broad_dashed")))
        return LineMarking::broad_dashed;
    else if (!(strcmp(type, "no_marking")))
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

