//
// Created by sebastian on 13.11.20.
//

#include "position_predicates.h"

#include <utility>

bool PositionPredicates::onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle>& obstacle){
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::mainCarriageWay;
    for(const auto& la : occupiedLanelets){
        if(std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(), [type](LaneletType t){return t == type;}))
            return true;
    }
    return false;
}

bool PositionPredicates::onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle>& obstacle){
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::mainCarriageWay;
    for(const auto& la : occupiedLanelets){
        if(std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(), [type](LaneletType t){return t == type;})
        and (!(la->getAdjacentRight().dir != DrivingDirection::same
        or !std::any_of(la->getAdjacentRight().adj->getLaneletType().begin(),
                        la->getAdjacentRight().adj->getLaneletType().end(), [type](LaneletType t){return t == type;}))))
            return true;
    }
    return false;
}

bool PositionPredicates::onAccessRamp(int timeStep, const std::shared_ptr<Obstacle> &obstacle){
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::accessRamp;
    for(const auto& la : occupiedLanelets){
        if(std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(), [type](LaneletType t){return t == type;}))
            return true;
    }
    return false;
}

bool PositionPredicates::inFrontOf(int timeStep, const std::shared_ptr<Obstacle>& obsP, const std::shared_ptr<Obstacle>& obsK){
    if(obsP->frontS(timeStep) < obsK->rearS(timeStep)) {;
        return true;
    }
    else
        return false;
}

void PositionPredicates::setRoadNetwork(const std::shared_ptr<RoadNetwork> &net) {roadNetwork = net;}

PositionPredicates::PositionPredicates(std::shared_ptr<RoadNetwork> roadNetwork) : roadNetwork(std::move(roadNetwork)) {}
