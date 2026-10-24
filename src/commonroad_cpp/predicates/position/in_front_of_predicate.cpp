#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_front_of_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool InFrontOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::vector<std::string> &additionalFunctionParameters,
                                           bool setBased) {
    return robustEvaluation(timeStep, world, obstacleP, obstacleK, additionalFunctionParameters, setBased) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::vector<std::string> &additionalFunctionParameters,
                                           SetBasedMode mode) {
    return robustEvaluation(timeStep, world, obstacleP, obstacleK, additionalFunctionParameters, mode) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return robustEvaluation(lonPositionP, lonPositionK, lengthP, lengthK) > 0;
}

Constraint InFrontOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    return {obstacleP->frontS(world->getRoadNetwork(), timeStep) +
            0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength()};
}

Constraint InFrontOfPredicate::constraintEvaluation(double lonPositionP, double lengthK, double lengthP) {
    return {lonPositionP + 0.5 * lengthP + 0.5 * lengthK};
}

double InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    return obstacleK->rearS(
               timeStep,
               obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem(),
               setBased) -
           obstacleP->frontS(world->getRoadNetwork(), timeStep);
}

double InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            SetBasedMode mode) {
    if (!evaluatesOccupancy(obstacleK, timeStep, mode))
        return robustEvaluation(timeStep, world, obstacleP, obstacleK, additionalFunctionParameters,
                                mode != SetBasedMode::none);
    const auto ccsP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    // must: the rear of the occupancy is in front of the pth vehicle, i.e., all covered behaviors are in front of it;
    // may: the front of the occupancy is in front of the pth vehicle, i.e., at least one covered behavior is in front
    const double lonPositionK{mode == SetBasedMode::must ? obstacleK->rearS(timeStep, ccsP, true)
                                                         : obstacleK->frontS(timeStep, ccsP, true)};
    return lonPositionK - obstacleP->frontS(world->getRoadNetwork(), timeStep);
}

double InFrontOfPredicate::robustEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return (lonPositionK - 0.5 * lengthK) - (lonPositionP + 0.5 * lengthP);
}

InFrontOfPredicate::InFrontOfPredicate() : CommonRoadPredicate(true) {}
