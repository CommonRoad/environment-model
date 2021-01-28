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
        if (stopLine == nullptr) //or stop sign not in reference traffic signs of lanelet
            continue;
        Eigen::Vector2d stopLineS = obs->getOwnLane()->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(
                stopLine->getPoints().at(0).x, stopLine->getPoints().at(0).y);

        //maybe check orientation as in BA
        if (stopLineS.x() - 1 < obs->frontS(timeStep) and obs->frontS(timeStep) < stopLineS.x()) {
            return true;
        }
    }
    return false;
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

bool Predicates::atRedStraightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs){
    return atRedTrafficLight(timeStep, obs, TrafficLightDirection::straight);
}

bool Predicates::atRedLeftTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs){
    return atRedTrafficLight(timeStep, obs, TrafficLightDirection::left);
}

bool Predicates::atRedRightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs){
    return atRedTrafficLight(timeStep, obs, TrafficLightDirection::right);
}

bool Predicates::atRedTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs, TrafficLightDirection turnDir){
    std::vector<TrafficLightDirection> relevantTrafficLightDirections;
    switch(turnDir) {
        case TrafficLightDirection::left:
            relevantTrafficLightDirections = {TrafficLightDirection::left, TrafficLightDirection::leftStraight,
                                              TrafficLightDirection::leftRight};
            break;
        case TrafficLightDirection::right:
            relevantTrafficLightDirections = {TrafficLightDirection::leftRight, TrafficLightDirection::straightRight,
                                              TrafficLightDirection::straightRight};
            break;
        default:
            relevantTrafficLightDirections = {TrafficLightDirection::straight, TrafficLightDirection::leftStraight,
                                              TrafficLightDirection::straightRight};
    }
    auto activeTl { activeTrafficLights(timeStep, obs) };
    for (const auto &tl : activeTl){
        auto trafficLightState { tl->getElementAtTime(timeStep).color };
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [tl](const TrafficLightDirection& relevantDirection) { return relevantDirection == tl->getDirection(); }) and trafficLightState != TrafficLightState::green)
            return true;
    }

    // use all when no other relevant TL is active
    for (const auto &tl : activeTl){
        auto trafficLightState { tl->getElementAtTime(timeStep).color };
        if (tl->getDirection() == TrafficLightDirection::all and trafficLightState != TrafficLightState::green)
            return true;
    }

    return false;

}

std::set<std::shared_ptr<TrafficLight>> Predicates::activeTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs){
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    TrafficLightState inactive = TrafficLightState::inactive;
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        for (const auto &tl : la->getTrafficLights()) {
            if (tl->isActive() and tl->getElementAtTime(timeStep).color != inactive)
                trafficLights.insert(tl);
        }
    }
    return trafficLights;
}

bool Predicates::atGreenArrow(int timeStep, const std::shared_ptr<Obstacle> &obs){
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    std::string trafficSignID { TrafficSignElement::mapTrafficSignNameToCountryID("green_arrow", country) };
    for (const auto &la : lanelets) {
        auto trafficSigns { la->getTrafficSigns() };
        for (const auto &ts : trafficSigns){
            if (std::any_of(ts->getTrafficSignElement().begin(), ts->getTrafficSignElement().end(),
                            [trafficSignID](const TrafficSignElement& t) { return t.getId() == trafficSignID; }))
                return true;
        }

    }
    return false;
}

bool Predicates::onRightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs){
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getRightOutgoings().begin(), obs->getRightOutgoings().end(), [la](const std::shared_ptr<Lanelet>& outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}

bool Predicates::onLeftOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs){
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getLeftOutgoings().begin(), obs->getLeftOutgoings().end(), [la](const std::shared_ptr<Lanelet>& outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}

bool Predicates::onStraightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs){
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getStraightOutgoings().begin(), obs->getStraightOutgoings().end(), [la](const std::shared_ptr<Lanelet>& outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}
