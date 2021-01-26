//
// Created by Sebastian Maierhofer on 25.01.21.
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

bool Predicates::inSameLane(int timeStep,
                            const std::shared_ptr<Obstacle> &obsP,
                            const std::shared_ptr<Obstacle> &obsK) {
    for (const auto &laneP : obsP->getOccupiedLanes(roadNetwork, timeStep)) {
        for (const auto &laneK : obsK->getOccupiedLanes(roadNetwork, timeStep)) {
            if (laneP->getLanelet().getId() == laneK->getLanelet().getId())
                return true;
        }
    }
    return false;
}

bool Predicates::stopLineInFront(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        std::shared_ptr<StopLine> stopLine{la->getStopLine()};
        if (stopLine == NULL) //or stop sign not in reference traffic signs of lanelet
            continue;
        Eigen::Vector2d stopLineS = obs->getOwnLane()->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
                stopLine->getPoints().at(0).x, stopLine->getPoints().at(0).y);

        //maybe check orientation as in BA
        if (stopLineS.x() - 1 < obs->frontS(timeStep) and obs->frontS(timeStep) < stopLineS.x())
            return true;
    }
    return false;
}

bool Predicates::onLeftIncoming(int timeStep,
                                const std::shared_ptr<Obstacle> &obsP,
                                const std::shared_ptr<Obstacle> &obsK) {

}


bool Predicates::intersectionRegulatedByTrafficSigns(int timeStep, const std::shared_ptr<Obstacle> &obs) {

}

bool Predicates::intersectionRegulatedByTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs) {

}

bool Predicates::inSameIntersection(int timeStep, const std::shared_ptr<Obstacle> &obsP,
                                    const std::shared_ptr<Obstacle> &obsK) {

}

bool Predicates::inIntersectionMainArea(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    LaneletType type = LaneletType::intersection;
    for (const auto &la : lanelets) {
        if (std::any_of(la->getLaneletType().begin(), la->getLaneletType().end(),
                        [type](LaneletType t) { return t == type; }))
            return true;
    }
    return false;
}

