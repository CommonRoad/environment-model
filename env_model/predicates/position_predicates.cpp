//
// Created by sebastian on 13.11.20.
//

#include "position_predicates.h"
#include "../auxiliaryDefs/types_and_definitions.h"

bool onMainCarriageWay(int timeStep, std::shared_ptr<Obstacle> obstacle, RoadNetwork roadNetwork){
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = roadNetwork.getOccupiedLanelets(obstacle, 0);
    LaneletType type = LaneletType::mainCarriageWay;
    for(auto la : occupiedLanelets){
        if(std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(), [type](LaneletType t){return t == type;}))
            return true;
    }
}