//
// Created by Sebastian Maierhofer on 25.01.21.
//

#include "predicates.h"
#include <tuple>

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

bool Predicates::atRedStraightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    return atRedTrafficLight(timeStep, obs, TurningDirections::straight);
}

bool Predicates::atRedLeftTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    return atRedTrafficLight(timeStep, obs, TurningDirections::left);
}

bool Predicates::atRedRightTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    return atRedTrafficLight(timeStep, obs, TurningDirections::right);
}

bool Predicates::atRedTrafficLight(int timeStep, const std::shared_ptr<Obstacle> &obs, TurningDirections turnDir) {
    if (!onIncoming(timeStep, obs))
        return false;
    std::vector<TurningDirections> relevantTrafficLightDirections;
    switch (turnDir) {
        case TurningDirections::left:
            relevantTrafficLightDirections = {TurningDirections::left, TurningDirections::leftStraight,
                                              TurningDirections::leftRight};
            break;
        case TurningDirections::right:
            relevantTrafficLightDirections = {TurningDirections::leftRight, TurningDirections::straightRight,
                                              TurningDirections::straightRight};
            break;
        default:
            relevantTrafficLightDirections = {TurningDirections::straight, TurningDirections::leftStraight,
                                              TurningDirections::straightRight};
    }
    auto activeTl{activeTrafficLights(timeStep, obs)};
    for (const auto &tl : activeTl) {
        auto trafficLightState{tl->getElementAtTime(timeStep).color};
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [tl](const TurningDirections &relevantDirection) {
                            return relevantDirection == tl->getDirection();
                        }) and trafficLightState != TrafficLightState::green)
            return true;
    }

    // use all when no other relevant TL is active
    const TurningDirections tlDirectionAll{TurningDirections::all};
    const TrafficLightState tlStateGreen{TrafficLightState::green};
    if (std::any_of(activeTl.begin(), activeTl.end(), [timeStep](const std::shared_ptr<TrafficLight>& tl) {
        return tlDirectionAll == tl->getDirection() and tl->getElementAtTime(timeStep).color != tlStateGreen;
    }))
        return true;

    return false;

}

std::set<std::shared_ptr<TrafficLight>>
Predicates::activeTrafficLights(int timeStep, const std::shared_ptr<Obstacle> &obs) {
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

bool Predicates::atGreenArrow(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    if (!onIncoming(timeStep, obs))
        return false;
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    std::string trafficSignID{TrafficSignElement::mapTrafficSignNameToCountryID("green_arrow", country)};
    for (const auto &la : lanelets) {
        auto trafficSigns{la->getTrafficSigns()};
        for (const auto &ts : trafficSigns) {
            if (std::any_of(ts->getTrafficSignElements().begin(), ts->getTrafficSignElements().end(),
                            [trafficSignID](const std::shared_ptr<TrafficSignElement> &t) { return t->getId() == trafficSignID; }))
                return true;
        }

    }
    return false;
}

bool Predicates::onRightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getRightOutgoings().begin(), obs->getRightOutgoings().end(),
                        [la](const std::shared_ptr<Lanelet> &outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}

bool Predicates::onLeftOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getLeftOutgoings().begin(), obs->getLeftOutgoings().end(),
                        [la](const std::shared_ptr<Lanelet> &outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}

bool Predicates::onStraightOutgoing(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &la : lanelets) {
        if (std::any_of(obs->getStraightOutgoings().begin(), obs->getStraightOutgoings().end(),
                        [la](const std::shared_ptr<Lanelet> &outgoing) { return la->getId() == outgoing->getId(); }))
            return true;
    }
    return false;
}

bool Predicates::onIncoming(int timeStep, const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    for (const auto &inter : roadNetwork->getIntersections()) {
        for (const auto &incom : inter->getIncomings()) {
            for (const auto &la : lanelets)
                if (std::any_of(incom->getIncomingLanelets().begin(), incom->getIncomingLanelets().end(),
                                [la](const std::shared_ptr<Lanelet> &incomingLanelet) {
                                    return la->getId() == incomingLanelet->getId();
                                })) {
                    obs->setLeftOutgoings(incom->getLeftOutgoings());
                    obs->setStraightOutgoings(incom->getStraightOutgoings());
                    obs->setRightOutgoings(incom->getRightOutgoings());
                    return true;
                }
        }
    }
    return false;
}


int Predicates::getPriority(int timeStep, const std::shared_ptr<Obstacle> &obs, TurningDirections dir){
    auto lanelets{obs->getOccupiedLanelets(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lanelet>> relevantIncomingLanelets;
    for (const auto &la : lanelets) {
        auto incomingLanelets { incomingLaneletOfLanelet(la) };
        relevantIncomingLanelets.insert(relevantIncomingLanelets.end(), incomingLanelets.begin(), incomingLanelets.end());
    }
    auto prioTrafficSign { extractPriorityTrafficSign(relevantIncomingLanelets) };
    return priorityTable.at(prioTrafficSign->getId()).at(static_cast<int>(dir));
}

bool Predicates::hasPriority(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP, TurningDirections dirK, TurningDirections dirP) {
    return getPriority(timeStep, obsK, dirK) > getPriority(timeStep, obsP, dirP);
}

std::vector<std::shared_ptr<Lanelet>> Predicates::incomingLaneletOfLanelet(const std::shared_ptr<Lanelet>& la) {
    std::vector<std::shared_ptr<Lanelet>> incomingLanelets;
    if (la->hasLaneletType(LaneletType::intersection)){
        auto pre = la->getPredecessors().at(0);    //TODO currently only a single predecessor is considered
        while (!pre->hasLaneletType(LaneletType::incoming)){
            pre = pre->getPredecessors().at(0);
        }
        incomingLanelets.push_back(pre);
    } else if (la->hasLaneletType(LaneletType::incoming)) {
        incomingLanelets.push_back(la);
    }
    return incomingLanelets;
}

std::vector<std::string> getRelevantPrioritySignIDs() {
    std::vector<std::string> keys;
    keys.reserve(priorityTable.size());
    for(const auto& [key, value] : priorityTable) {
        keys.push_back(key);
    }
    return keys;
}

std::shared_ptr<TrafficSignElement> Predicates::extractPriorityTrafficSign(const std::vector<std::shared_ptr<Lanelet>>& lanelets){
    for ( const auto &la : lanelets) {  // TODO don't use just first value
        auto ts{extractPriorityTrafficSign(la)};
        if (ts == nullptr)
            continue;
        else
            return ts;
    }
}

std::shared_ptr<TrafficSignElement> Predicates::extractPriorityTrafficSign(const std::shared_ptr<Lanelet>& lanelet){
    static const std::vector<std::string> relevantPrioritySignIds { getRelevantPrioritySignIDs() };
    std::vector<std::shared_ptr<TrafficSignElement>> relevantTrafficSignElements;
    for (const auto &ts : lanelet->getTrafficSigns()){
        auto trafficSignElements { ts->getTrafficSignElements() };
        relevantTrafficSignElements.insert(relevantTrafficSignElements.end(), trafficSignElements.begin(), trafficSignElements.end());
    }
    for (const auto &tse : relevantTrafficSignElements){
        if (std::any_of(relevantTrafficSignElements.begin(), relevantTrafficSignElements.end(), [tse](const std::shared_ptr<TrafficSignElement> &el){return el->getId() == tse->getId();}))
            continue;
        else
            return tse; //TODO don't just return first sign -> look at BA there it is different
    }
    return nullptr;
}

std::vector<std::shared_ptr<Lanelet>> Predicates::findUpcomingIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>>& lanelets) {
    std::vector<std::shared_ptr<Lanelet>> upcomingIncomingLanelets;
    for (const auto &la : lanelets)
        if (la->hasLaneletType(LaneletType::incoming))
            upcomingIncomingLanelets.push_back(la);
    if (!upcomingIncomingLanelets.empty())
        return upcomingIncomingLanelets;

    for (const auto &la : lanelets) {
        auto upcomingLanelets{findUpcomingIncomingLanelets(la)};
        if (upcomingLanelets.empty())
            upcomingIncomingLanelets.insert(upcomingLanelets.end(), upcomingLanelets.begin(), upcomingLanelets.end());
    }
    return upcomingIncomingLanelets;
}

std::vector<std::shared_ptr<Lanelet>> Predicates::findUpcomingIncomingLanelets(const std::shared_ptr<Lanelet>& lanelet){
    std::vector<std::shared_ptr<Lanelet>> upcomingIncomingLanelets;
    if (lanelet->hasLaneletType(LaneletType::incoming)) {
        upcomingIncomingLanelets.push_back(lanelet);
        return upcomingIncomingLanelets;
    }
    auto succs = lanelet->getSuccessors();
    while (!succs.empty()){
        std::vector<std::shared_ptr<Lanelet>> upcomingIncomingLaneletsTmp;
        for (const auto &suc : succs){
            if (suc->hasLaneletType(LaneletType::incoming))
                upcomingIncomingLanelets.push_back(suc);
            else if (!suc->getSuccessors().empty())
                upcomingIncomingLaneletsTmp.insert(upcomingIncomingLaneletsTmp.end(), suc->getSuccessors().begin(), suc->getSuccessors().end());
        }
        succs = upcomingIncomingLaneletsTmp;
    }
    return upcomingIncomingLanelets;
}

std::vector<std::shared_ptr<Lanelet>> Predicates::incomingLaneletsLeftOfLanelet(const std::shared_ptr<Lanelet> &lanelet) {

}

bool Predicates::isLeftOf(int timeStep, const std::shared_ptr<Obstacle> &obsK, const std::shared_ptr<Obstacle> &obsP){
    for (const auto &laneK : obsK->getOccupiedLanes(roadNetwork, timeStep)){
        for (const auto &laneP : obsP->getOccupiedLanes(roadNetwork, timeStep)){
            for (const auto &laneletK : laneK->getContainedLanelets()){
                auto incomingLaneletsK { roadNetwork->incomingOfLanelet(laneletK) };
                if (incomingLaneletsK != nullptr) {
                    for (const auto &laneletP : laneP->getContainedLanelets()){
                        if (std::any_of(incomingLaneletsK->getIncomingLanelets().begin(), incomingLaneletsK->getIncomingLanelets().end(),
                                        [laneletP](const std::shared_ptr<Lanelet> &la){ return la->getId() == laneletP->getId(); })) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

