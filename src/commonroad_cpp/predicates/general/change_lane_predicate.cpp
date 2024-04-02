#include "commonroad_cpp/predicates/general/change_lane_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/world.h"

#include <stdexcept>

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
            if (std::any_of(
                    occupiedLaneletsBack.begin(), occupiedLaneletsBack.end(),
                    [&](const std::shared_ptr<Lanelet> &back) {
                        auto occDrivDir = obstacleK->getOccupiedLaneletsRoadByShape(world->getRoadNetwork(), timeStep);
                        if (back->getAdjacentLeft().adj &&
                            // check whether lanelet occupied by the back of the car has a left adjacent which is
                            // occupied by the front of the car
                            back->getAdjacentLeft().adj->getId() == front->getId() &&
                            // check if back is also in the lanelts, which are occupied and in driving direction,
                            // to avoid false positive if changing to the right coming from oncoming lanelet
                            std::any_of(occDrivDir.begin(), occDrivDir.end(),
                                        [back](const auto &occ) { return back->getId() == occ->getId(); }))
                            return true;
                        return false;
                    }))
                return true;
        }
    } // Check if vehicle is changing to the right adjacent
    else if (additionalFunctionParameters->turningDirection.front() == TurningDirection::right) {
        for (const auto &front : occupiedLaneletsFront) {
            if (std::any_of(
                    occupiedLaneletsBack.begin(), occupiedLaneletsBack.end(),
                    [&](const std::shared_ptr<Lanelet> &back) {
                        auto occ = obstacleK->getOccupiedLaneletsRoadByShape(world->getRoadNetwork(), timeStep);
                        // check whether lanelet occupied by the back of the car has a right adjacent which is
                        // occupied by the front of the car
                        if (back->getAdjacentRight().adj && back->getAdjacentRight().adj->getId() == front->getId())
                            return true;
                        // Catch false negative if changing to the right if coming from oncoming lanelet
                        if (back->getAdjacentLeft().adj && back->getAdjacentLeft().adj->getId() == front->getId() &&
                            std::none_of(occ.begin(), occ.end(),
                                         [back](const auto &occ) { return back->getId() == occ->getId(); }))
                            return true;
                        return false;
                    }))
                return true;
        }
    } else {
        throw std::invalid_argument(
            "Invalid TurningDirection for change_lanelet_predicate. Only 'left' and 'right' are allowed");
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
