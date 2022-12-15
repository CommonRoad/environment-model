#include "commonroad_cpp/predicates/position/close_to_intersection_predicate.h"

#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

CloseToIntersectionPredicate::CloseToIntersectionPredicate() : CommonRoadPredicate(false) {}

bool CloseToIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);

    std::map<size_t, double> result_ids_with_distance;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const std::shared_ptr<Lanelet> &lanelet : occupiedLanelets) {
        auto left_end_of_lane = lanelet->getLeftBorderVertices().back();
        result_ids_with_distance.insert({lanelet->getId(), obstacle_operations::drivingDistanceToCoordinatePoint(
                                                               left_end_of_lane.x, left_end_of_lane.y,
                                                               world->getRoadNetwork(), obstacleK, timeStep)});
        lanelets.push_back(lanelet);
    }
    while (!lanelets.empty()) {
        std::vector<std::shared_ptr<Lanelet>> tmp_succs;
        // iterates all lanes in lanelets. If one is of the type "incoming", check the distance to the end and return
        // true if it is smaller than the max distance to intersections
        for (const std::shared_ptr<Lanelet> &lane : lanelets) {
            if (lane->hasLaneletType(LaneletType::incoming)) {
                if (result_ids_with_distance.find(lane->getId())->second <
                    parameters.close_to_intersection_max_distance)
                    return true;
            } else {
                if (!lane->getSuccessors().empty()) {
                    for (const std::shared_ptr<Lanelet> &successor : lane->getSuccessors()) {
                        double distance =
                            result_ids_with_distance.find(lane->getId())->second + successor->getPathLength().back();
                        if (result_ids_with_distance.find(successor->getId()) == result_ids_with_distance.end() and
                            distance < parameters.close_to_intersection_max_distance) {
                            tmp_succs.push_back(successor);
                            result_ids_with_distance.insert({successor->getId(), distance});
                        }
                    }
                }
            }
        }
        lanelets = tmp_succs;
    }
    return false;
}

double CloseToIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Close To intersection Predicate does not support robust evaluation!");
}

Constraint CloseToIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Close To intersection Predicate does not support constraint evaluation!");
}
