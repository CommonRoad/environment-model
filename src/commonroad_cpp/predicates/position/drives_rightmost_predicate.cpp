#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/world.h"

#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/position/drives_rightmost_predicate.h>

bool DrivesRightmostPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_right =
        obstacle_operations::obstacleDirectlyRight(timeStep, world->getObstacles(), obstacleK, world->getRoadNetwork());

    auto referenceLaneK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
    if (vehicle_directly_right != nullptr and
        referenceLaneK->getCurvilinearCoordinateSystem()->cartesianPointInProjectionDomain(
            obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
            obstacleK->getStateByTimeStep(timeStep)->getYPosition()) and
        (obstacleK->rightD(world->getRoadNetwork(), timeStep) -
         vehicle_directly_right->leftD(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep))) <
            parameters.getParam("closeToOtherVehicle")) {
        return true;
    } else {
        std::vector<std::shared_ptr<Lane>> lanes{obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep)};
        return std::all_of(lanes.begin(), lanes.end(), [obstacleK, this, timeStep](const std::shared_ptr<Lane> &lane) {
            return lane->getCurvilinearCoordinateSystem()->cartesianPointInProjectionDomain(
                       obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                       obstacleK->getStateByTimeStep(timeStep)->getYPosition()) and
                   0.5 * lane->getWidth(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                        obstacleK->getStateByTimeStep(timeStep)->getYPosition()) +
                           obstacleK->rightD(timeStep, lane) <=
                       parameters.getParam("closeToLaneBorder");
        });
    }
}

double DrivesRightmostPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Drives Rightmost Predicate does not support robust evaluation!");
}

Constraint DrivesRightmostPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Drives Rightmost Predicate does not support constraint evaluation!");
}

DrivesRightmostPredicate::DrivesRightmostPredicate() : CommonRoadPredicate(false) {}
