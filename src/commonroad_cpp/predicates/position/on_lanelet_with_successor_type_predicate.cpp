#include "commonroad_cpp/predicates/position/on_lanelet_with_successor_type_predicate.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

OnLaneletWithSuccessorTypePredicate::OnLaneletWithSuccessorTypePredicate() : CommonRoadPredicate(false) {}

bool OnLaneletWithSuccessorTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);

    for (auto &lanelet : occupiedLanelets) {
        if (lanelet->hasLaneletType(additionalFunctionParameters->laneletType.at(0)))
            return true;
        auto left_end_of_lane = lanelet->getLeftBorderVertices().back();
        double dist_to_end_of_lane = obstacle_operations::drivingDistanceToCoordinatePoint(
            left_end_of_lane.x, left_end_of_lane.y, world->getRoadNetwork(), obstacleK, timeStep);
        // Reduce fov by dist_to_end_of_lane because the distance of the initial lane will be neglected
        std::vector<std::vector<std::shared_ptr<Lanelet>>> successors =
            lanelet_operations::combineLaneletAndSuccessorsToLane(lanelet, obstacleK->getFieldOfViewFrontDistance() -
                                                                               dist_to_end_of_lane);
        for (auto &vec : successors) {
            for (auto &lane : vec) {
                if (lane->hasLaneletType(additionalFunctionParameters->laneletType.at(0)))
                    return true;
            }
        }
    }
    return false;
}

double OnLaneletWithSuccessorTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("On Lanelet With Successor Type Predicate does not support robust evaluation!");
}

Constraint OnLaneletWithSuccessorTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("On Lanelet With Successor Type Predicate does not support constraint evaluation!");
}
