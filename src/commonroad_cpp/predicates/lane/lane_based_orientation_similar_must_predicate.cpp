#include <commonroad_cpp/predicates/lane/lane_based_orientation_similar_must_predicate.h>

bool LaneBasedOrientationSimilarMustPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    if (evaluatesOccupancy(obstacleK, timeStep, setBased))
        return false;
    return LaneBasedOrientationSimilarPredicate::booleanEvaluation(timeStep, world, obstacleP, obstacleK,
                                                                   additionalFunctionParameters, setBased);
}
