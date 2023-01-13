#include <cmath>
#include <stdexcept>

#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_neighboring_right_lane_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h>
#include <commonroad_cpp/predicates/braking/safe_distance_gap_right_violated_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool SafeDistanceGapRightViolatedPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    InNeighboringRightLanePredicate inNeighboringRightLanePredicate;
    KeepsSafeDistancePrecPredicate KeepsSafeDistancePrecPredicate;
    std::shared_ptr<OptionalPredicateParameters> predicateParameters =
        std::make_shared<OptionalPredicateParameters>(OptionalPredicateParameters(parameters.minSafetyDistance));
    bool safeDistanceViolated = false;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (inNeighboringRightLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs)) {
            InFrontOfPredicate inFrontOfPredicate;
            std::shared_ptr<Obstacle> rightObstacle = obstacle_operations::obstacleDirectlyRight(
                timeStep, world->getObstacles(), obstacleK, world->getRoadNetwork());
            if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) &&
                !KeepsSafeDistancePrecPredicate.booleanEvaluation(timeStep, world, obstacleK, obs,
                                                                  predicateParameters)) {
                // obs in front of k
                safeDistanceViolated = true;
                break;
            } else if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obs, obstacleK) &&
                       !KeepsSafeDistancePrecPredicate.booleanEvaluation(timeStep, world, obs, obstacleK,
                                                                         predicateParameters)) {
                // k in front of obs
                safeDistanceViolated = true;
                break;
            } else if (rightObstacle != nullptr) {
                safeDistanceViolated = true;
                break;
            }
        }
    }
    return safeDistanceViolated;
}

Constraint SafeDistanceGapRightViolatedPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support constrained evaluation!");
}

double SafeDistanceGapRightViolatedPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SafeDistanceGapRightViolatedPredicate does not support robust evaluation!");
}

SafeDistanceGapRightViolatedPredicate::SafeDistanceGapRightViolatedPredicate() : CommonRoadPredicate(false) {}
