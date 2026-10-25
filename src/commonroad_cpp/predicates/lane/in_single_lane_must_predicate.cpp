#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_single_lane_must_predicate.h>
#include <commonroad_cpp/world.h>

bool InSingleLaneMustPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    if (!evaluatesOccupancy(obstacleK, timeStep, setBased))
        return InSingleLanePredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP,
                                                        additionalFunctionParameters, setBased);
    return isSingleLane(obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, true));
}
