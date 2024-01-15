#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/right_of_predicate.h>

bool RightOfPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Obstacle>> allObs{world->getObstacles()};
    allObs.emplace_back(obstacleK);
    auto rightObstacles{obstacle_operations::obstaclesRight(timeStep, allObs, obstacleP, world->getRoadNetwork())};
    return std::any_of(rightObstacles.begin(), rightObstacles.end(), [obstacleK](const std::shared_ptr<Obstacle> &obs) {
        return obstacleK->getId() == obs->getId();
    });
}

double
RightOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Obstacle> &obstacleK,
                                   const std::shared_ptr<Obstacle> &obstacleP,
                                   const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RightOfPredicate does not support robust evaluation!");
}

Constraint RightOfPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RightOfPredicate does not support constraint evaluation!");
}

RightOfPredicate::RightOfPredicate() : CommonRoadPredicate(true) {}
