#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_must_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

double KeepsSafeDistancePrecMustPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    if (!evaluatesOccupancy(obstacleP, timeStep, setBased))
        return KeepsSafeDistancePrecPredicate::robustEvaluation(timeStep, world, obstacleK, obstacleP,
                                                                additionalFunctionParameters, setBased);
    const auto ccsK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    const double frontK{obstacleK->frontS(world->getRoadNetwork(), timeStep)};
    const double frontP{obstacleP->frontS(timeStep, ccsK, true)};
    // safe distance not applicable to covered behaviors behind the kth vehicle; collision must be checked separately
    if (frontP < frontK)
        return frontK - frontP;
    const double deltaS{obstacleP->rearS(timeStep, ccsK, true) - frontK};
    const double dSafe{computeSafeDistance(obstacleK->getVelocity(timeStep, false),
                                           obstacleP->getVelocity(timeStep, true, true), obstacleK->getAminLong(),
                                           obstacleP->getAminLong(), obstacleK->getReactionTime())};
    const double param0 = std::stod(additionalFunctionParameters.at(0));
    if (deltaS - param0 < 0)
        return std::min(deltaS - param0, deltaS - dSafe);
    return deltaS - dSafe;
}
