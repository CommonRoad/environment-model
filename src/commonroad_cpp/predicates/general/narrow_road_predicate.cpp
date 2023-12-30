#include "commonroad_cpp/predicates/general/narrow_road_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/world.h"

bool NarrowRoadPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    double obsK_x = obstacleK->getStateByTimeStep(timeStep)->getXPosition();
    double obsK_y = obstacleK->getStateByTimeStep(timeStep)->getYPosition();
    auto occupied_lanelets = world->getRoadNetwork()->findLaneletsByPosition(obsK_x, obsK_y);
    return std::all_of(occupied_lanelets.begin(), occupied_lanelets.end(),
                       [obsK_x, obsK_y, world, this](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet_operations::roadWidth(lanelet, obsK_x, obsK_y) <= parameters.narrowRoad;
                       });
}

Constraint NarrowRoadPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Narrow Road Predicate does not support constraint evaluation!");
}

double NarrowRoadPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Narrow Road Predicate does not support robust evaluation!");
}

NarrowRoadPredicate::NarrowRoadPredicate() : CommonRoadPredicate(false) {}