#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h"
#include "commonroad_cpp/predicates/position/drives_rightmost_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/predicates/position/overtaking_bicycle_same_lane_predicate.h>

bool OvertakingBicycleSameLanePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    InSameLanePredicate inSameLanePredicate;
    InFrontOfPredicate inFrontOfPredicate;
    DrivesRightmostPredicate drivesRightmostPredicate;
    KeepsSafeDistancePrecPredicate KeepsSafeDistancePrecPredicate;
    bool overtakingBicycle = false;
    std::shared_ptr<OptionalPredicateParameters> predicateParameters =
        std::make_shared<OptionalPredicateParameters>(OptionalPredicateParameters(parameters.minSafetyDistance));
    std::shared_ptr<Obstacle> rightObstacle =
        obstacle_operations::obstacleDirectlyRight(timeStep, world->getObstacles(), obstacleK, world->getRoadNetwork());

    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (obs->getObstacleType() == ObstacleType::bicycle &&
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) &&
            ((((inFrontOfPredicate.robustEvaluation(timeStep, world, obstacleK, obs) > 0.0 &&
                inFrontOfPredicate.robustEvaluation(timeStep, world, obstacleK, obs) < parameters.closeToBicycle) ||
               (inFrontOfPredicate.booleanEvaluation(timeStep, world, obs, obstacleK) &&
                !KeepsSafeDistancePrecPredicate.booleanEvaluation(timeStep, world, obs, obstacleK,
                                                                  predicateParameters))) &&
              !drivesRightmostPredicate.booleanEvaluation(timeStep, world, obstacleK)) ||
             rightObstacle == obs)) {
            overtakingBicycle = true;
            break;
        }
    }
    return overtakingBicycle;
}

double OvertakingBicycleSameLanePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Overtaking Bicycle Predicate does not support robust evaluation!");
}

Constraint OvertakingBicycleSameLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Overtaking Bicycle Predicate does not support constraint evaluation!");
}
OvertakingBicycleSameLanePredicate::OvertakingBicycleSameLanePredicate() : CommonRoadPredicate(false) {}