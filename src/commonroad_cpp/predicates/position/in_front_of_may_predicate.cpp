#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_front_of_may_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

double InFrontOfMayPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {
    if (!evaluatesOccupancy(obstacleK, timeStep, setBased))
        return InFrontOfPredicate::robustEvaluation(timeStep, world, obstacleP, obstacleK, additionalFunctionParameters,
                                                    setBased);
    // not defined if the vehicles are not on a common reference lane: not in front (robustness -inf)
    return predicate_utils::valueIfUndefined(undefinedRobustness(), [&] {
        const auto ccsP{
            obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
        return obstacleK->frontS(timeStep, ccsP, true) - obstacleP->frontS(world->getRoadNetwork(), timeStep);
    });
}

bool InFrontOfMayPredicate::undefinedValue() const { return true; }
