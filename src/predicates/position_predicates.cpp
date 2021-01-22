//
// Created by Sebastian Maierhofer on 13.11.20.
//

#include "predicates.h"

bool Predicates::onMainCarriageWay(int timeStep, const std::shared_ptr<Obstacle> &obstacle) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::mainCarriageWay;
    for (const auto &la : occupiedLanelets) {
        if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                        [type](LaneletType t) { return t == type; }))
            return true;
    }
    return false;
}

bool Predicates::onMainCarriageWayRightLane(int timeStep, const std::shared_ptr<Obstacle> &obstacle) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::mainCarriageWay;
    for (const auto &la : occupiedLanelets) {
        if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                        [type](LaneletType t) { return t == type; })
            and (la->getAdjacentRight().dir == DrivingDirection::invalid
                 or la->getAdjacentRight().dir != DrivingDirection::same
                 or !std::any_of(la->getAdjacentRight().adj->getLaneletType().begin(),
                                 la->getAdjacentRight().adj->getLaneletType().end(),
                                 [type](LaneletType t) { return t == type; })))
            return true;
    }
    return false;
}

bool Predicates::onAccessRamp(int timeStep, const std::shared_ptr<Obstacle> &obstacle) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacle->getOccupiedLanelets(roadNetwork, timeStep);
    LaneletType type = LaneletType::accessRamp;
    for (const auto &la : occupiedLanelets) {
        if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                        [type](LaneletType t) { return t == type; }))
            return true;
    }
    return false;
}

bool Predicates::inFrontOf(int timeStep,
                                   const std::shared_ptr<Obstacle> &obsP,
                                   const std::shared_ptr<Obstacle> &obsK) {
    if (obsP->frontS(timeStep) < obsK->rearS(timeStep))
        return true;
    else
        return false;
}

bool Predicates::inSameLane(int timeStep,
                           const std::shared_ptr<Obstacle> &obsP,
                           const std::shared_ptr<Obstacle> &obsK) {
    for (const auto &laneP : obsP->getOccupiedLanes(roadNetwork, timeStep)){
        for (const auto &laneK : obsK->getOccupiedLanes(roadNetwork, timeStep)){
            if (laneP->getLanelet().getId() == laneK->getLanelet().getId())
                return true;
        }
    }

    return false;
}

void Predicates::setRoadNetwork(const std::shared_ptr<RoadNetwork> &net) { roadNetwork = net; }


