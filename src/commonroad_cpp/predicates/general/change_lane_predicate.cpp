#include "commonroad_cpp/predicates/general/change_lane_predicate.h"

bool ChangeLanePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLaneletsFront =
        obstacleK->getOccupiedLaneletsByFront(world->getRoadNetwork(), timeStep);
    std::vector<std::shared_ptr<Lanelet>> occupiedLaneletsBack =
        obstacleK->getOccupiedLaneletsByBack(world->getRoadNetwork(), timeStep);

    // Check if the whole vehicle is within one lane to avoid "true" if the vehicle is in a branching lane
    for (const auto &front : occupiedLaneletsFront) {
        if (std::any_of(occupiedLaneletsBack.begin(), occupiedLaneletsBack.end(),
                        [front](const std::shared_ptr<Lanelet> &back) {
                            return front->getId() == back->getId() or
                                   std::any_of(front->getPredecessors().begin(), front->getPredecessors().end(),
                                               [back](const std::shared_ptr<Lanelet> &predecessor) {
                                                   return predecessor->getId() == back->getId();
                                               }) or
                                   std::any_of(back->getSuccessors().begin(), back->getSuccessors().end(),
                                               [front](const std::shared_ptr<Lanelet> &successor) {
                                                   return successor->getId() == front->getId();
                                               });
                        }))
            return false;
    }
    // Check if vehicle is changing to the left adjacent
    if (additionalFunctionParameters->turningDirection.front() == TurningDirection::left) {
        for (const auto &front : occupiedLaneletsFront) {
            if (std::any_of(occupiedLaneletsBack.begin(), occupiedLaneletsBack.end(),
                            [front, additionalFunctionParameters](const std::shared_ptr<Lanelet> &back) {
                                if (back->getAdjacentLeft().adj)
                                    return back->getAdjacentLeft().adj->getId() == front->getId();
                            }))
                return true;
        }
    } // Check if vehicle is changing to the right adjacent
    else if (additionalFunctionParameters->turningDirection.front() == TurningDirection::right) {
        for (const auto &front : occupiedLaneletsFront) {
            if (std::any_of(occupiedLaneletsBack.begin(), occupiedLaneletsBack.end(),
                            [front, additionalFunctionParameters](const std::shared_ptr<Lanelet> &back) {
                                if (back->getAdjacentRight().adj)
                                    return back->getAdjacentRight().adj->getId() == front->getId();
                            }))
                return true;
        }
    } else {
        throw std::invalid_argument(
            "Ivalid TurningDirection for change_lanelet_predicate. Only 'left' and 'right' are allowed");
    }
    return false;
}

Constraint ChangeLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("ChangeLane Predicate does not support constraint evaluation!");
}

double ChangeLanePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("ChangeLane Predicate does not support robust evaluation!");
}

ChangeLanePredicate::ChangeLanePredicate() : CommonRoadPredicate(false) {}
