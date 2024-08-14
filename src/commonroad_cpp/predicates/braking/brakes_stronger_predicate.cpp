#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/brakes_stronger_predicate.h>
#include <commonroad_cpp/world.h>

bool BrakesStrongerPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters) > 0;
}

Constraint BrakesStrongerPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    return {
        std::min(obstacleP->getStateByTimeStep(timeStep)->getAcceleration(), stod(additionalFunctionParameters.at(0)))};
}

double BrakesStrongerPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters) {
    return std::min(obstacleP->getStateByTimeStep(timeStep)->getAcceleration(),
                    stod(additionalFunctionParameters.at(0))) -
           obstacleK->getStateByTimeStep(timeStep)->getAcceleration();
}

BrakesStrongerPredicate::BrakesStrongerPredicate() : CommonRoadPredicate(true) {}
