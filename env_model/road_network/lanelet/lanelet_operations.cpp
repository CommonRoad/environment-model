//
// Created by Sebastian Maierhofer on 09.11.20.
//

#include "lanelet_operations.h"

LaneletType matchStringToLaneletType(const char *type){
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

LineMarking matchStringToLineMarking(const char *type){
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

std::shared_ptr<Lanelet> combineLaneletAndSuccessorsWithSameType(Lanelet* curLanelet, LaneletType type) {
    // assumption: all successors have different type
    std::shared_ptr<Lanelet> newLanelet;
    std::vector<vertice> centerVertices;
    std::vector<vertice> leftBorder;
    std::vector<vertice> rightBorder;
    std::vector<Lanelet *> predecessorLanelets;
    std::shared_ptr<Lanelet> successorLanelet{nullptr};

    while (curLanelet != nullptr){
        for(auto la : curLanelet->getSuccessors()) {
            for (const auto &sucType : la->getLaneletType()) {
                if (sucType == type) {
                    curLanelet = la.get();
                    break;
                }
            }
        }
    }
}
