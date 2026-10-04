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
    const auto ccsP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    // For a set-based prediction of the kth vehicle, the predicate is used as may-version, i.e., it holds if at least
    // one behavior covered by the occupancy is in front of the pth vehicle. This is the case if the frontmost point of
    // the occupancy is in front of the pth vehicle's front. The may-version is required for soundness since the
    // predicate appears negated in rule preconditions, e.g., in succeeds of R_G1.
    if (setBased and !obstacleK->getSetBasedPrediction().empty() and
        timeStep > obstacleK->getCurrentState()->getTimeStep())
        return obstacleK->frontS(timeStep, ccsP, setBased) - obstacleP->frontS(world->getRoadNetwork(), timeStep);
    return obstacleK->rearS(timeStep, ccsP, setBased) - obstacleP->frontS(world->getRoadNetwork(), timeStep);
}

double InFrontOfPredicate::robustEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return (lonPositionK - 0.5 * lengthK) - (lonPositionP + 0.5 * lengthP);
}

InFrontOfPredicate::InFrontOfPredicate() : CommonRoadPredicate(true) {}
