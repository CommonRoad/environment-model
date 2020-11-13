//
// Created by sebastian on 13.11.20.
//

#include "position_predicates.h"

bool PositionPredicates::onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle>& obstacle, const RoadNetwork& roadNetwork){
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork.getLaneletNetwork(), timeStep);
    LaneletType type = LaneletType::mainCarriageWay;
    for(const auto& la : occupiedLanelets){
        if(std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(), [type](LaneletType t){return t == type;}))
            return true;
    }
    return false;
}