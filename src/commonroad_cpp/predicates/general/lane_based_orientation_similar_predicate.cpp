#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.h>
#include <commonroad_cpp/world.h>
#include <stdexcept>

bool LaneBasedOrientationSimilarPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto referenceLaneP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    if (!referenceLaneP->getCurvilinearCoordinateSystem()->cartesianPointInProjectionDomain(
            obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
            obstacleK->getStateByTimeStep(timeStep)->getYPosition()) or
        !referenceLaneP->getCurvilinearCoordinateSystem()->cartesianPointInProjectionDomain(
            obstacleP->getStateByTimeStep(timeStep)->getXPosition(),
            obstacleP->getStateByTimeStep(timeStep)->getYPosition()))
        return false;
    return std::abs(geometric_operations::subtractOrientations(
               obstacleK->getCurvilinearOrientation(timeStep, referenceLaneP),
               obstacleP->getCurvilinearOrientation(world->getRoadNetwork(), timeStep))) <
           parameters.getParam("laneMatchingOrientation");
}

Constraint LaneBasedOrientationSimilarPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Lane Based Orientation Similar does not support constraint evaluation!");
}

double LaneBasedOrientationSimilarPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Lane Based Orientation Similar does not support robust evaluation!");
}
LaneBasedOrientationSimilarPredicate::LaneBasedOrientationSimilarPredicate() : CommonRoadPredicate(true) {}
