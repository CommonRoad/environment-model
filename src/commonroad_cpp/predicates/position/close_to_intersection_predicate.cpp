#include "commonroad_cpp/predicates/position/close_to_intersection_predicate.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

CloseToIntersectionPredicate::CloseToIntersectionPredicate() : CommonRoadPredicate(false) {}

bool CloseToIntersectionPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);

    std::map<size_t, double> resultIdsWithDistance;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const std::shared_ptr<Lanelet> &lanelet : occupiedLanelets) {
        auto centerEndOfLane = lanelet->getCenterVertices().back();
        if (!obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
                 ->getCurvilinearCoordinateSystem()
                 ->cartesianPointInProjectionDomain(centerEndOfLane.x, centerEndOfLane.y) or
            !obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
                 ->getCurvilinearCoordinateSystem()
                 ->cartesianPointInProjectionDomain(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                    obstacleK->getStateByTimeStep(timeStep)->getYPosition()))
            continue;
        double distToEnd = obstacle_operations::drivingDistanceToCoordinatePoint(
            centerEndOfLane.x, centerEndOfLane.y, world->getRoadNetwork(), obstacleK, timeStep);
        resultIdsWithDistance.insert({lanelet->getId(), distToEnd});
        lanelets.push_back(lanelet);
    }
    while (!lanelets.empty()) {
        std::vector<std::shared_ptr<Lanelet>> tmp_succs;
        // iterates all lanes in lanelets. If one is of the type "incoming", check the distance to the end and return
        // true if it is smaller than the max distance to intersections
        for (const std::shared_ptr<Lanelet> &lane : lanelets) {
            if (lane->hasLaneletType(LaneletType::incoming)) {
                if (resultIdsWithDistance.find(lane->getId())->second <
                    parameters.getParam("closeToIntersectionMaxDistance"))
                    return true;
            } else {
                if (!lane->getSuccessors().empty()) {
                    for (const std::shared_ptr<Lanelet> &successor : lane->getSuccessors()) {
                        double distance =
                            resultIdsWithDistance.find(lane->getId())->second + successor->getPathLength().back();
                        if (resultIdsWithDistance.find(successor->getId()) == resultIdsWithDistance.end() and
                            distance < parameters.getParam("closeToIntersectionMaxDistance")) {
                            tmp_succs.push_back(successor);
                            resultIdsWithDistance.insert({successor->getId(), distance});
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
