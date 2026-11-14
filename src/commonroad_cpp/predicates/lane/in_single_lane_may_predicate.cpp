#include <commonroad_cpp/predicates/lane/in_single_lane_may_predicate.h>

bool InSingleLaneMayPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    if (evaluatesOccupancy(obstacleK, timeStep, setBased))
        return true;
    return InSingleLanePredicate::booleanEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters,
                                                    setBased);
}
