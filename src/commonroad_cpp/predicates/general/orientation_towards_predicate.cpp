#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/general/orientation_towards_predicate.h>

bool OrientationTowardsPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    auto ccsP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    const auto curvilinearOrientation{obstacleK->getCurvilinearOrientation(timeStep, ccsP)};
    const auto latPositionP{obstacleP->getLatPosition(world->getRoadNetwork(), timeStep)};
    const auto latPositionK{obstacleK->getLatPosition(timeStep, ccsP)};
    return (latPositionK > latPositionP and curvilinearOrientation < 0) or // k on left side
           (latPositionK < latPositionP and curvilinearOrientation > 0);   // k on right side
}

double OrientationTowardsPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("OrientationTowardsPredicate does not support robust evaluation!");
}

Constraint OrientationTowardsPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OrientationTowardsPredicate does not support constraint evaluation!");
}
OrientationTowardsPredicate::OrientationTowardsPredicate() : CommonRoadPredicate(true) {}
