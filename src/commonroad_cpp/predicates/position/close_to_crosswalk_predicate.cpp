#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/position/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>

#include <commonroad_cpp/predicates/position/close_to_crosswalk_predicate.h>
#include <valarray>

bool CloseToCrosswalkPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters) {

    auto occNotDrivDir = obstacleK->getOccupiedLaneletsNotDrivingDirectionByShape(world->getRoadNetwork(), timeStep);
    auto occDrivDir = obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);

    std::map<size_t, double> resultIdsWithDistance;
    std::vector<std::shared_ptr<Lanelet>> lanelets;

    for (const std::shared_ptr<Lanelet> &lanelet : occDrivDir) {
        auto centerEndOfLane = lanelet->getCenterVertices().back();
        double distToEnd = obstacle_operations::drivingDistanceToCoordinatePoint(
            centerEndOfLane.x, centerEndOfLane.y, world->getRoadNetwork(), obstacleK, timeStep);
        resultIdsWithDistance.insert({lanelet->getId(), distToEnd});
        lanelets.push_back(lanelet);
    }
    for (const std::shared_ptr<Lanelet> &lanelet : occNotDrivDir) {
        std::shared_ptr<Lanelet> tmp;
        if (lanelet->getAdjacentLeft().adj && lanelet->getAdjacentLeft().oppositeDir)
            tmp = lanelet->getAdjacentLeft().adj;
        else if (lanelet->getAdjacentRight().adj && lanelet->getAdjacentRight().oppositeDir)
            tmp = lanelet->getAdjacentRight().adj;
        else
            continue;
        auto centerEndOfLane = tmp->getCenterVertices().back();
        double distToEnd = obstacle_operations::drivingDistanceToCoordinatePoint(
            centerEndOfLane.x, centerEndOfLane.y, world->getRoadNetwork(), obstacleK, timeStep);
        resultIdsWithDistance.insert({tmp->getId(), distToEnd});
        lanelets.push_back(tmp);
    }

    while (!lanelets.empty()) {
        std::vector<std::shared_ptr<Lanelet>> tmp_succs;
        // iterates all lanes in lanelets. If one is of the type "incoming", check the distance to the end and return
        // true if it is smaller than the max distance to intersections
        for (const std::shared_ptr<Lanelet> &lane : lanelets) {
            if (lane->hasTrafficSign(TrafficSignTypes::PEDESTRIANS_CROSSING)) {
                if (resultIdsWithDistance.find(lane->getId())->second < parameters.getParam("dCloseToCrossing"))
                    return true;
            } else {
                if (!lane->getSuccessors().empty()) {
                    for (const std::shared_ptr<Lanelet> &successor : lane->getSuccessors()) {
                        double distance =
                            resultIdsWithDistance.find(lane->getId())->second + successor->getPathLength().back();
                        if (resultIdsWithDistance.find(successor->getId()) == resultIdsWithDistance.end() and
                            distance < parameters.getParam("dCloseToCrossing")) {
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
double CloseToCrosswalkPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CloseToCrosswalkPredicate does not support robust evaluation!");
}
Constraint CloseToCrosswalkPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CloseToCrosswalkPredicate does not support constraint evaluation!");
}
CloseToCrosswalkPredicate::CloseToCrosswalkPredicate() : CommonRoadPredicate(false) {}
