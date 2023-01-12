#include "commonroad_cpp/predicates/position/oncoming_vehicle_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_neighboring_left_lane_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

bool OncomingVehiclePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    bool oncomingTraffic = false;
    InFrontOfPredicate inFrontOfPredicate;
    InNeighboringLeftLanePredicate inNeighboringLeftLanePredicate;
    InSameLanePredicate inSameLanePredicate;

    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if ((inFrontOfPredicate.booleanEvaluation(timeStep, world, obs, obstacleK)) &&
            inNeighboringLeftLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) &&
            !inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) &&
            (obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)[0]->getAdjacentLeft().dir ==
                 DrivingDirection::opposite &&
             obs->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)[0]->getAdjacentLeft().dir ==
                 DrivingDirection::opposite)) {
            oncomingTraffic = true;
            break;
        }
    }
    return oncomingTraffic;
}

double OncomingVehiclePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Is Oncoming Vehicle Predicate does not support robust evaluation!");
}

Constraint OncomingVehiclePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Is Oncoming Vehicle Predicate does not support constraint evaluation!");
}
OncomingVehiclePredicate::OncomingVehiclePredicate() : CommonRoadPredicate(false) {}
